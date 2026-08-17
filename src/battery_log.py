"""Always-on CSV trace of pack voltage / current / SoC.

Kept separate from ``battery.py`` so that module stays pure and offline-testable
-- this is the only part that touches the filesystem.

Why always-on rather than a deliberate "discharge run" mode: the pack will
almost certainly die in the middle of something else, and a capture you have to
remember to start is a capture you will not have.  One file per server start
means each power cycle is already segmented into its own discharge curve.

Why fsync: the interesting event *is* the power cut.  A plain write only reaches
the OS page cache, which a battery cutout discards -- up to 30 s of the most
important samples.  At the default 10 s cadence an fsync'd append costs about
6 writes per minute, which is nothing next to the 20 Hz telemetry loop it rides
along with.
"""

from __future__ import annotations

import os
import json
import time
import glob
import logging

logger = logging.getLogger(__name__)

HEADER = "iso_time,epoch_s,voltage_v,current_a,power_w,soc_pct\n"


class BatteryLogger:
    """Append-only CSV logger, self-throttling and failure-tolerant.

    Call :meth:`log` as often as convenient (it ignores calls inside the
    interval).  Any I/O error disables the logger permanently rather than
    propagating -- losing the trace must never take telemetry down with it.
    """

    def __init__(self, directory: str, interval_s: float = 10.0,
                 keep: int = 30, sync: bool = True):
        self.directory = directory
        self.interval_s = interval_s
        self.keep = keep
        self.sync = sync
        self.path: str | None = None
        self._fh = None
        self._last_t: float | None = None
        self._rows = 0
        self._failed = False

    # -- lifecycle ----------------------------------------------------------
    def _open(self) -> bool:
        try:
            os.makedirs(self.directory, exist_ok=True)
            self._prune()
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.path = os.path.join(self.directory, f"battery_{stamp}.csv")
            new = not os.path.exists(self.path)
            self._fh = open(self.path, "a")
            if new:
                self._fh.write(HEADER)
                self._fh.flush()
            logger.info(f"BatteryLogger: writing {self.path} every {self.interval_s:g}s")
            return True
        except Exception as e:
            logger.error(f"BatteryLogger: disabled, could not open log: {e}")
            self._failed = True
            return False

    def _prune(self) -> None:
        """Keep only the newest ``keep`` traces so this cannot grow forever."""
        try:
            files = sorted(glob.glob(os.path.join(self.directory, "battery_*.csv")))
            for old in files[: max(0, len(files) - self.keep + 1)]:
                os.remove(old)
        except Exception as e:
            logger.warning(f"BatteryLogger: prune failed (continuing): {e}")

    def _note_failure(self, where: str, exc: BaseException) -> None:
        """Record why logging stopped, somewhere journald cannot rate-limit away.

        The service emits enough output to trip journald's rate limiter, so an
        error logged the normal way can vanish exactly when it matters.
        """
        import traceback
        try:
            with open(os.path.join(self.directory, "logger_errors.txt"), "a") as fh:
                fh.write(f"--- {time.strftime('%Y-%m-%dT%H:%M:%S')} {where}\n")
                fh.write(f"{type(exc).__name__}: {exc}\n")
                fh.write(traceback.format_exc() + "\n")
        except Exception:
            pass

    # -- input --------------------------------------------------------------
    def log(self, voltage: float, current: float, soc_pct: float,
            now: float | None = None) -> bool:
        """Record one sample if the interval has elapsed.  Returns True if written."""
        if self._failed:
            return False
        now = time.time() if now is None else now
        if self._last_t is not None and (now - self._last_t) < self.interval_s:
            return False
        if self._fh is None and not self._open():
            return False
        try:
            iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now))
            self._fh.write(
                f"{iso},{now:.3f},{voltage:.4f},{current:.4f},"
                f"{voltage * current:.4f},{soc_pct:.2f}\n"
            )
            self._fh.flush()
            if self.sync:
                os.fsync(self._fh.fileno())
            self._last_t = now
            self._rows += 1
            return True
        except Exception as e:
            logger.error(f"BatteryLogger: disabled after write error: {e}")
            self._note_failure("write", e)
            self._failed = True
            return False

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
                if self.sync:
                    os.fsync(self._fh.fileno())
                self._fh.close()
            except Exception:
                pass
            self._fh = None


class SoCState:
    """Persist the coulomb counter across restarts.

    Coulomb counting has no way to recover its own zero, and on this pack the
    OCV curve cannot supply one above the knee -- the whole 30-100% band is
    300 mV wide.  So a server restart mid-session would otherwise reset the
    gauge to whatever the plateau happens to imply, which is the exact error
    this estimator exists to avoid.  The x3_server restarts often enough
    (nine traces across two days) that this is the common case, not an edge one.

    Kept here rather than in ``battery.py`` so that module stays free of I/O.
    """

    #: Beyond this age the stored charge is not trusted -- the pack may have
    #: been charged, swapped, or simply relaxed far from where we left it.
    MAX_AGE_S = 12 * 3600.0

    #: A pack whose voltage came back up by more than this while we were not
    #: looking gained charge from somewhere, so the stored value is stale.
    #: Sized above ordinary post-load relaxation on the plateau.
    CHARGE_DETECT_V = 0.15

    def __init__(self, path: str, min_write_interval_s: float = 60.0):
        self.path = path
        self.min_write_interval_s = min_write_interval_s
        self._last_write: float | None = None
        self._failed = False

    def load(self, voltage: float, now: float | None = None) -> float | None:
        """Return a usable stored ``charge_ah``, or None to re-seed from OCV.

        ``voltage`` is the pack voltage observed right now, used to notice that
        the pack was charged while the server was down.
        """
        now = time.time() if now is None else now
        try:
            with open(self.path) as fh:
                data = json.load(fh)
            charge = float(data["charge_ah"])
            stamp = float(data["epoch_s"])
            stored_v = float(data["voltage_v"])
        except FileNotFoundError:
            return None
        except Exception as e:
            logger.warning(f"SoCState: ignoring unreadable state file: {e}")
            return None

        age = now - stamp
        if age < 0 or age > self.MAX_AGE_S:
            logger.info(f"SoCState: stored charge is {age / 3600.0:.1f} h old, re-seeding from OCV")
            return None
        if voltage > stored_v + self.CHARGE_DETECT_V:
            logger.info(
                f"SoCState: pack rose {voltage - stored_v:.2f} V while down "
                f"(charged or swapped), re-seeding from OCV"
            )
            return None
        logger.info(f"SoCState: restored {charge:.3f} Ah from {age / 60.0:.1f} min ago")
        return charge

    def save(self, charge_ah: float, voltage: float, now: float | None = None) -> bool:
        """Store the counter, at most once per ``min_write_interval_s``.

        Written via a temp file and rename so a power cut cannot leave a
        half-written state file behind -- the one failure mode that would make
        this worse than having no persistence at all.
        """
        if self._failed or charge_ah is None:
            return False
        now = time.time() if now is None else now
        if self._last_write is not None and (now - self._last_write) < self.min_write_interval_s:
            return False
        tmp = self.path + ".tmp"
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            with open(tmp, "w") as fh:
                json.dump({"charge_ah": round(charge_ah, 4), "epoch_s": round(now, 3),
                           "voltage_v": round(voltage, 4)}, fh)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self.path)
            self._last_write = now
            return True
        except Exception as e:
            logger.error(f"SoCState: disabled after write error: {e}")
            self._failed = True
            return False
