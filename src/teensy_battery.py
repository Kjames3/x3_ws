"""X3 v2 pack monitor: the Teensy 4.1 battery monitor's CSV over a UART.

src/battery_monitor/battery_monitor.ino owns the INA226 and the state of
charge for the 2 x 10 Ah LiFePO4 pack (coulomb counting with full / rested-OCV
/ empty-knee anchors, persisted in its EEPROM). It prints one line per 250 ms
on USB *and* on Serial1 (TX1, pin 1, 115200 8N1):

    pct,volts,amps,ah_remaining,state        e.g.  "82.4,13.104,1.032,16.480,DISCHARGING"

amps are positive DISCHARGING (the server's convention too), state is
CHARGING / DISCHARGING / IDLE, and lines starting with '#' are diagnostics.

So the server does not estimate SoC for this pack -- battery.BatteryEstimator
models the old 4S 8.07 Ah pack -- it only reads, validates and relays.

A bare UART has no checksum, and a flipped bit in a digit still parses. Each
line is therefore cross-checked: five fields, plausible ranges, the state that
the sketch's own thresholds give for that current, and percent consistent with
ah_remaining / capacity. A line that fails is counted and dropped.

Hardware note: TX1 -> the AGX 40-pin UART RX with a common ground. On the AGX
Orin dev kit that header UART should be /dev/ttyTHS1 (there is no ttyTHS0;
ttyAMA0 is the boot console), but it is NOT verified -- loop pins 8/10 or
watch the port with `python3 src/teensy_battery.py /dev/ttyTHS1` first. The
Teensy's USB serial carries the same lines, so a USB port works as well.
"""
import logging
import math
import threading
import time
from dataclasses import dataclass

logger = logging.getLogger("x3_server")

DEFAULT_PORT = "/dev/ttyTHS1"
BAUD = 115200
CAPACITY_AH = 20.0          # battery_monitor.ino CAPACITY_AH (2 x 10 Ah in parallel)
STATE_I = 0.2               # the sketch: i < -0.2 CHARGING, i > 0.2 DISCHARGING, else IDLE
STALE_S = 3.0               # 12 missed lines at 4 Hz
STATES = ("CHARGING", "DISCHARGING", "IDLE")


@dataclass(frozen=True)
class BatterySample:
    pct: float
    volts: float
    amps: float             # + discharging
    ah_remaining: float
    state: str
    t: float                # time.monotonic() at receipt


def parse_line(line, t=None, capacity_ah=CAPACITY_AH):
    """One CSV line -> BatterySample, or None for a comment or a bad line.
    Raises nothing: UART noise is expected, not exceptional."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            return None
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != 5:
        return None
    try:
        pct, volts, amps, ah = (float(p) for p in parts[:4])
    except ValueError:
        return None
    state = parts[4]
    if not all(math.isfinite(x) for x in (pct, volts, amps, ah)):
        return None
    if not (0.0 <= pct <= 100.0 and 8.0 <= volts <= 16.0 and abs(amps) <= 60.0
            and 0.0 <= ah <= capacity_ah * 1.05):
        return None
    if state not in STATES:
        return None
    expect = "CHARGING" if amps < -STATE_I else "DISCHARGING" if amps > STATE_I else "IDLE"
    if state != expect:
        return None
    # The sketch derives pct from ah_remaining (pct = 100 * ah / capacity) after
    # its anchors, so the two agree to print rounding.
    if abs(pct - 100.0 * ah / capacity_ah) > 1.0:
        return None
    return BatterySample(pct, volts, amps, ah, state, time.monotonic() if t is None else t)


class TeensyBatteryMonitor:
    """Reader thread on the Teensy's UART; INA226BatteryMonitor-shaped getters.

    Getters return the newest valid sample's values, or None when nothing
    valid has arrived for STALE_S, so the caller falls back explicitly."""

    def __init__(self, port=DEFAULT_PORT, baud=BAUD, capacity_ah=CAPACITY_AH, start=True):
        self.port = port
        self.baud = baud
        self.capacity_ah = capacity_ah
        self.connected = False
        self.lines_ok = 0
        self.lines_rejected = 0
        self._latest = None
        self._discharge_a = None        # EMA of discharge current for minutes_remaining
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        if start:
            self._thread = threading.Thread(target=self._run, name="teensy-battery", daemon=True)
            self._thread.start()

    # ------------------------------------------------------------- reading
    def feed(self, line, t=None):
        """Handle one received line (the reader thread calls this; tests too)."""
        s = parse_line(line, t, self.capacity_ah)
        if s is None:
            text = line.decode("ascii", "replace") if isinstance(line, (bytes, bytearray)) else line
            if text.strip() and not text.lstrip().startswith("#"):
                self.lines_rejected += 1
            return None
        with self._lock:
            prev = self._latest
            self._latest = s
            self.lines_ok += 1
            if s.amps > STATE_I:
                # ~60 s time constant at 4 Hz: motor bursts don't swing the estimate.
                if self._discharge_a is None:
                    self._discharge_a = s.amps
                else:
                    dt = 0.25 if prev is None else max(0.0, min(s.t - prev.t, 5.0))
                    k = 1.0 - math.exp(-dt / 60.0)
                    self._discharge_a += k * (s.amps - self._discharge_a)
            elif s.amps < -STATE_I:
                self._discharge_a = None
        return s

    def _run(self):
        try:
            import serial
        except ImportError:
            logger.warning("pyserial not installed: Teensy battery monitor unavailable")
            return
        logged_missing = False
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.5, exclusive=True) as ser:
                    self.connected = True
                    logged_missing = False
                    logger.info(f"Teensy battery monitor: reading {self.port}")
                    ser.reset_input_buffer()          # drop a half line from before open
                    while not self._stop.is_set():
                        raw = ser.readline()
                        if raw:
                            self.feed(raw)
            except Exception as e:
                if self.connected or not logged_missing:
                    logger.warning(f"Teensy battery monitor on {self.port}: {e}; retrying every 2 s")
                    logged_missing = True
            self.connected = False
            self._stop.wait(2.0)

    def cleanup(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------- getters
    def latest(self, now=None):
        """Newest valid sample, or None if it is older than STALE_S."""
        with self._lock:
            s = self._latest
        now = time.monotonic() if now is None else now
        if s is None or now - s.t > STALE_S:
            return None
        return s

    def get_voltage(self):
        s = self.latest()
        return s.volts if s else None

    def get_current(self):
        s = self.latest()
        return s.amps if s else None

    def get_power(self):
        s = self.latest()
        return s.volts * s.amps if s else None

    def get_percentage(self, voltage=None):
        """The Teensy's coulomb-counted SoC; `voltage` is ignored (kept for the
        INA226BatteryMonitor signature)."""
        s = self.latest()
        return s.pct if s else None

    def minutes_remaining(self, now=None):
        """Time to empty at the smoothed discharge current; None unless discharging."""
        s = self.latest(now)
        with self._lock:
            a = self._discharge_a
        if s is None or a is None or s.state != "DISCHARGING" or a <= STATE_I:
            return None
        return 60.0 * s.ah_remaining / a

    def status(self):
        s = self.latest()
        return {"port": self.port, "connected": self.connected, "fresh": s is not None,
                "lines_ok": self.lines_ok, "lines_rejected": self.lines_rejected,
                "state": s.state if s else None, "ah_remaining": s.ah_remaining if s else None}


if __name__ == "__main__":      # bench check: python3 src/teensy_battery.py [port]
    import sys
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    mon = TeensyBatteryMonitor(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT)
    try:
        while True:
            time.sleep(1.0)
            s = mon.latest()
            m = mon.minutes_remaining()
            print(f"{'---' if s is None else f'{s.pct:5.1f}% {s.volts:6.3f} V {s.amps:+7.3f} A {s.ah_remaining:6.3f} Ah {s.state}'}"
                  f"  min_left={m if m is None else round(m)}  ok={mon.lines_ok} rejected={mon.lines_rejected}"
                  f"  connected={mon.connected}")
    except KeyboardInterrupt:
        mon.cleanup()
