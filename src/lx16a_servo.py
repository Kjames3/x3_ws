#!/usr/bin/env python3
"""Minimal LewanSoul/HiWonder LX-16A bus-servo driver.

The LX-16A speaks a half-duplex 115200 8N1 protocol on a single signal wire, so
it needs a proper bus adapter (the CH340 debug board on /dev/lx16a) -- the
Rosmaster's S1-S4 headers are PWM-only and cannot reach it.

Frame layout:  0x55 0x55 <id> <len> <cmd> [params...] <checksum>
    len      = len(params) + 3
    checksum = ~(id + len + cmd + sum(params)) & 0xFF

Positions are raw servo counts: 0..1000 spans 240 degrees (4.1667 counts/deg).
That is NOT the Yahboom YB-SD15M scale Rosmaster_Lib assumes (12.2 counts/deg),
so never route LX-16A counts through Rosmaster_Lib's angle conversion.
"""

import time

try:
    import serial
except ImportError:  # keeps --help usable on machines without pyserial
    serial = None

DEFAULT_PORT = "/dev/lx16a"
BAUD = 115200

COUNTS_PER_DEG = 1000.0 / 240.0   # 4.1667
DEG_PER_COUNT = 240.0 / 1000.0    # 0.24

# command ids
MOVE_TIME_WRITE = 1
MOVE_TIME_READ = 2
MOVE_STOP = 12
ID_READ = 14
ANGLE_OFFSET_READ = 19
ANGLE_LIMIT_READ = 21
VIN_LIMIT_READ = 23
TEMP_READ = 26
VIN_READ = 27
POS_READ = 28
MODE_READ = 30
LOAD_WRITE = 31
LOAD_READ = 32

BROADCAST_ID = 254


class LX16AError(Exception):
    pass


class LX16A:
    """Blocking, synchronous LX-16A client. One instance per serial port."""

    def __init__(self, port=DEFAULT_PORT, baud=BAUD, timeout=0.05, retries=2):
        if serial is None:
            raise LX16AError("pyserial is not installed")
        self.port = port
        self.retries = retries
        self._ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- framing -------------------------------------------------------
    @staticmethod
    def _frame(servo_id, cmd, params=b""):
        length = len(params) + 3
        body = bytes([servo_id, length, cmd]) + bytes(params)
        chk = (~sum(body)) & 0xFF
        return b"\x55\x55" + body + bytes([chk])

    def send(self, servo_id, cmd, params=b""):
        """Fire-and-forget write (no reply expected)."""
        self._ser.reset_input_buffer()
        self._ser.write(self._frame(servo_id, cmd, params))
        self._ser.flush()

    def _read_frames(self, deadline):
        """Collect whole frames from the port until `deadline`."""
        buf = bytearray()
        frames = []
        while time.monotonic() < deadline:
            chunk = self._ser.read(32)
            if chunk:
                buf.extend(chunk)
            elif buf:
                pass
            # parse out every complete frame we have
            while True:
                start = buf.find(b"\x55\x55")
                if start < 0 or len(buf) - start < 6:
                    break
                length = buf[start + 3]
                total = start + 3 + length          # 2 header + id + len..chk
                if len(buf) < total:
                    break
                frame = bytes(buf[start:total])
                del buf[:total]
                body = frame[2:-1]
                if (~sum(body)) & 0xFF == frame[-1]:
                    frames.append(frame)
            if frames:
                return frames
            if not chunk:
                time.sleep(0.002)
        return frames

    def query(self, servo_id, cmd, n_params):
        """Send a read command and return its parameter bytes.

        Some adapters echo the outgoing frame back on the shared wire, so
        replies are matched on (id, cmd, param count) rather than on order.
        """
        want_len = n_params + 3
        last_err = None
        for _ in range(self.retries + 1):
            self.send(servo_id, cmd)
            deadline = time.monotonic() + 0.06
            for frame in self._read_frames(deadline):
                fid, flen, fcmd = frame[2], frame[3], frame[4]
                if fcmd != cmd or flen != want_len:
                    continue          # echo of our own request, or crosstalk
                if servo_id != BROADCAST_ID and fid != servo_id:
                    continue
                return fid, frame[5:5 + n_params]
            last_err = "no reply from id %d to cmd %d" % (servo_id, cmd)
        raise LX16AError(last_err)

    # ---- reads ---------------------------------------------------------
    @staticmethod
    def _s16(b):
        v = b[0] | (b[1] << 8)
        return v - 65536 if v > 32767 else v

    def read_pos(self, servo_id):
        """Current *physical* position in counts (can go outside 0..1000)."""
        return self._s16(self.query(servo_id, POS_READ, 2)[1])

    def read_target(self, servo_id):
        """(target counts, move time ms) of the last move command."""
        p = self.query(servo_id, MOVE_TIME_READ, 4)[1]
        return self._s16(p[0:2]), p[2] | (p[3] << 8)

    def read_temp(self, servo_id):
        return self.query(servo_id, TEMP_READ, 1)[1][0]

    def read_vin(self, servo_id):
        p = self.query(servo_id, VIN_READ, 2)[1]
        return (p[0] | (p[1] << 8)) / 1000.0

    def read_offset(self, servo_id):
        v = self.query(servo_id, ANGLE_OFFSET_READ, 1)[1][0]
        return v - 256 if v > 127 else v

    def read_angle_limits(self, servo_id):
        p = self.query(servo_id, ANGLE_LIMIT_READ, 4)[1]
        return p[0] | (p[1] << 8), p[2] | (p[3] << 8)

    def read_mode(self, servo_id):
        """(mode, speed) -- mode 0 = servo (position), 1 = continuous motor."""
        p = self.query(servo_id, MODE_READ, 4)[1]
        return p[0], self._s16(p[2:4])

    def is_loaded(self, servo_id):
        """True when the motor is powered (holding position)."""
        return bool(self.query(servo_id, LOAD_READ, 1)[1][0])

    def ping(self, servo_id):
        try:
            return self.query(servo_id, ID_READ, 1)[0]
        except LX16AError:
            return None

    def scan(self, ids=range(0, 254)):
        found = []
        for i in ids:
            if self.ping(i) is not None:
                found.append(i)
        return found

    # ---- writes --------------------------------------------------------
    def set_load(self, servo_id, on):
        self.send(servo_id, LOAD_WRITE, bytes([1 if on else 0]))

    def move(self, servo_id, counts, time_ms=1000):
        counts = int(max(0, min(1000, counts)))
        time_ms = int(max(0, min(30000, time_ms)))
        self.send(servo_id, MOVE_TIME_WRITE, bytes([
            counts & 0xFF, (counts >> 8) & 0xFF,
            time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        ]))

    def stop(self, servo_id):
        self.send(servo_id, MOVE_STOP)
