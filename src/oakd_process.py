"""Parent-side proxy for the out-of-process OAK-D driver.

Drop-in replacement for OakDCamera: same constructor kwargs, same getters, same
status attributes. The real driver runs in oakd_child.py; this class only reads
finished frames out of shared memory.

Why: OakDCamera._run was the top CPU consumer in x3_server and it holds the GIL
while it runs (mm->m depth conversion, host-side YOLO decode), stalling the
asyncio event loop. Moving it to a separate process gives it its own interpreter
and its own GIL.

Reads are cached by sequence number, so N consumers of the same frame (velocity
estimator, ROS publisher, GUI depth view) cost exactly one memcpy per new frame.
"""
import logging
import os
import pickle
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

from oakd_shm import CAPACITIES, Slot, slot_names

logger = logging.getLogger(__name__)

# Match oakd_driver's colourisation so the GUI depth view is byte-identical.
try:
    from oakd_driver import DEPTH_MIN_M, DEPTH_MAX_M
except Exception:                                    # pragma: no cover
    DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 8.0

_HEARTBEAT_TIMEOUT = 2.0     # meta older than this => treat the child as down
_RESTART_BACKOFF_MAX = 10.0


class OakDCameraProcess:
    """Same API as OakDCamera, backed by a child process."""

    def __init__(self, sim_mode=False, **kwargs):
        self.sim_mode = sim_mode
        self._kwargs = dict(kwargs)
        self._kwargs.setdefault("sim_mode", False)
        self._prefix = f"x3oak{os.getpid()}"
        self._slots = {}
        self._proc = None
        self._running = False
        self._monitor = None

        self._lock = threading.Lock()
        self._meta_cache = {}
        self._meta_seq = -1
        self._depth_cache = None
        self._depth_seq = -1
        self._depth_stamp = 0.0
        self._depth_color = None
        self._left_cache = None
        self._left_seq = -1
        self._right_cache = None
        self._right_seq = -1

        # Mirrors of the driver's plain attributes, refreshed from meta.
        self._fx = self._fy = self._cx = self._cy = None

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        if self.sim_mode:
            logger.info("OakDCameraProcess: sim_mode -- child not started")
            return
        names = slot_names(self._prefix)
        try:
            for key, name in names.items():
                self._slots[key] = Slot(name, capacity=CAPACITIES[key], create=True)
        except Exception as e:
            logger.error(f"OakDCameraProcess: could not create shared memory: {e}")
            self._teardown_slots()
            raise
        self._running = True
        self._spawn_child()
        self._monitor = threading.Thread(target=self._monitor_loop,
                                         name="oak-proc-monitor", daemon=True)
        self._monitor.start()
        logger.info(f"OakDCameraProcess: child launched (prefix {self._prefix})")

    def _spawn_child(self):
        import json
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oakd_child.py")
        cmd = [sys.executable, script,
               "--prefix", self._prefix,
               "--kwargs-json", json.dumps(self._kwargs),
               "--parent-pid", str(os.getpid())]
        self._proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL)
        logger.info(f"OakDCameraProcess: child pid {self._proc.pid}")

    def _monitor_loop(self):
        backoff = 1.0
        while self._running:
            time.sleep(0.5)
            p = self._proc
            if p is None or p.poll() is None:
                backoff = 1.0
                continue
            if not self._running:
                break
            logger.error(f"OakDCameraProcess: child exited (rc={p.returncode}); "
                         f"restarting in {backoff:.0f}s")
            with self._lock:
                self._meta_cache = {}
                self._meta_seq = -1
            time.sleep(backoff)
            backoff = min(backoff * 2, _RESTART_BACKOFF_MAX)
            if self._running:
                try:
                    self._spawn_child()
                except Exception as e:
                    logger.error(f"OakDCameraProcess: restart failed: {e}")

    def cleanup(self):
        self._running = False
        p = self._proc
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                logger.warning("OakDCameraProcess: child ignored SIGTERM; killing")
                try:
                    p.kill()
                    p.wait(timeout=2.0)
                except Exception:
                    pass
            except Exception:
                pass
        m = self._monitor
        if m is not None:
            m.join(timeout=2.0)
        self._teardown_slots()
        logger.info("OakDCameraProcess: stopped")

    stop = cleanup

    def _teardown_slots(self):
        for s in self._slots.values():
            s.close()
            s.unlink()
        self._slots = {}

    # ------------------------------------------------------------------ meta
    def _meta(self):
        slot = self._slots.get("meta")
        if slot is None:
            return {}
        try:
            seq = slot.sequence()
        except Exception:
            return self._meta_cache
        with self._lock:
            if seq == self._meta_seq:
                return self._meta_cache
        got = slot.read_bytes()
        if not got:
            with self._lock:
                return self._meta_cache
        payload, s = got
        try:
            meta = pickle.loads(payload)
        except Exception:
            with self._lock:
                return self._meta_cache
        with self._lock:
            self._meta_cache, self._meta_seq = meta, s
            intr = meta.get("intrinsics")
            if intr:
                self._fx, self._fy, self._cx, self._cy = intr[0], intr[1], intr[2], intr[3]
            return meta

    def _meta_fresh(self):
        m = self._meta()
        hb = m.get("heartbeat")
        if hb is None or time.monotonic() - hb > _HEARTBEAT_TIMEOUT:
            return {}
        return m

    @property
    def available(self):
        return bool(self._meta_fresh().get("available", False))

    @property
    def spatial_active(self):
        return bool(self._meta_fresh().get("spatial_active", False))

    @property
    def economy(self):
        return bool(self._meta_fresh().get("economy", False))

    @property
    def usb_speed(self):
        return self._meta_fresh().get("usb_speed")

    @property
    def depth_fps(self):
        return float(self._meta_fresh().get("depth_fps", 0.0) or 0.0)

    @property
    def labels(self):
        return self._meta().get("labels", [])

    # ------------------------------------------------------------------ getters
    def get_frame(self):
        return None

    def get_raw_depth_frame(self):
        slot = self._slots.get("depth")
        if slot is None:
            return None
        try:
            seq = slot.sequence()
        except Exception:
            return None
        with self._lock:
            if seq == self._depth_seq:
                return self._depth_cache
        got = slot.read_array()
        if got is None:
            with self._lock:
                return self._depth_cache
        arr, s, stamp = got
        with self._lock:
            self._depth_cache, self._depth_seq, self._depth_stamp = arr, s, stamp
            self._depth_color = None
            return arr

    def get_depth_frame(self):
        raw = self.get_raw_depth_frame()
        with self._lock:
            if self._depth_color is not None:
                return self._depth_color
        if raw is None:
            return None
        coloured = self._colourise(raw)
        with self._lock:
            if self._depth_cache is raw:
                self._depth_color = coloured
        return coloured

    @staticmethod
    def _colourise(raw_m):
        clean = np.where(raw_m <= 0.1, DEPTH_MAX_M, raw_m)
        clean = np.clip(clean, DEPTH_MIN_M, DEPTH_MAX_M)
        min_d = float(clean.min())
        max_d = float(clean.max())
        if max_d > min_d:
            norm = (255.0 * (1.0 - (clean - min_d) / (max_d - min_d))).astype(np.uint8)
        else:
            norm = np.zeros_like(clean, dtype=np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_BONE)

    def get_depth_frame_age(self) -> float:
        self.get_raw_depth_frame()      # refresh the stamp if a new frame landed
        with self._lock:
            t = self._depth_stamp
        # time.monotonic() is CLOCK_MONOTONIC -- system-wide, so the child's stamp
        # is directly comparable here.
        return float("inf") if t == 0.0 else time.monotonic() - t

    def get_depth_fps(self) -> float:
        return self.depth_fps

    def get_stereo_frames(self):
        return self._read_mono("left"), self._read_mono("right")

    def _read_mono(self, key):
        slot = self._slots.get(key)
        if slot is None:
            return None
        cache_attr, seq_attr = f"_{key}_cache", f"_{key}_seq"
        try:
            seq = slot.sequence()
        except Exception:
            return None
        with self._lock:
            if seq == getattr(self, seq_attr):
                return getattr(self, cache_attr)
        got = slot.read_array()
        if got is None:
            with self._lock:
                return getattr(self, cache_attr)
        arr, s, _stamp = got
        with self._lock:
            setattr(self, cache_attr, arr)
            setattr(self, seq_attr, s)
            return arr

    def get_imu(self):
        return self._meta_fresh().get("imu")

    def get_spatial_detections(self):
        return list(self._meta_fresh().get("detections") or [])

    def get_depth_intrinsics(self):
        intr = self._meta_fresh().get("intrinsics")
        return tuple(intr) if intr else None
