#!/usr/bin/env python3
"""OAK-D capture worker -- runs the real OakDCamera in its own process.

Launched by OakDCameraProcess (oakd_process.py) as a plain subprocess, NOT via
multiprocessing: forking a parent that already holds ROS2 executor threads risks
inheriting locked mutexes, and `spawn` would re-import server_x3.py's module-level
argparse in the child. A standalone script sidesteps both.

Everything the parent used to do on its own GIL -- the mm->m depth conversion and
the host-side YOLO decode -- happens here instead. The parent only ever memcpys a
finished frame out of shared memory.
"""
import argparse
import json
import logging
import os
import pickle
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from oakd_shm import CAPACITIES, Slot, slot_names   # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [oak-child] %(levelname)s: %(message)s")
logger = logging.getLogger("oakd_child")

_running = True


def _stop(signum, frame):
    global _running
    _running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True, help="shared-memory name prefix")
    ap.add_argument("--kwargs-json", default="{}", help="OakDCamera constructor kwargs")
    ap.add_argument("--parent-pid", type=int, default=0)
    ap.add_argument("--meta-hz", type=float, default=50.0)
    args = ap.parse_args()

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    kwargs = json.loads(args.kwargs_json)
    names = slot_names(args.prefix)

    # The parent creates the blocks; attach (retry briefly in case we won the race).
    slots = {}
    for key, name in names.items():
        for attempt in range(50):
            try:
                slots[key] = Slot(name)
                break
            except FileNotFoundError:
                time.sleep(0.02)
        else:
            logger.error(f"could not attach shared-memory slot {name}")
            return 1

    from oakd_driver import OakDCamera
    cam = OakDCamera(**kwargs)
    cam.start()
    logger.info(f"started OakDCamera in child pid {os.getpid()} "
                f"(prefix {args.prefix}, kwargs {kwargs})")

    last_depth_t = None
    last_left = last_right = None
    last_meta = 0.0
    meta_period = 1.0 / max(args.meta_hz, 1.0)
    ppid = args.parent_pid or os.getppid()

    try:
        while _running:
            # Parent gone (crash / kill -9)? Do not linger holding the USB device.
            if ppid and os.getppid() != ppid and os.getppid() == 1:
                logger.warning("parent process exited; shutting down")
                break

            with cam._lock:
                depth_t = cam._last_depth_time
                raw = cam._latest_raw_depth
            if raw is not None and depth_t != last_depth_t:
                try:
                    slots["depth"].write_array(raw)
                    last_depth_t = depth_t
                except Exception as e:
                    logger.error(f"depth publish failed: {e}")

            left, right = cam.get_stereo_frames()
            if left is not None and left is not last_left:
                try:
                    slots["left"].write_array(left)
                    last_left = left
                except Exception as e:
                    logger.error(f"left publish failed: {e}")
            if right is not None and right is not last_right:
                try:
                    slots["right"].write_array(right)
                    last_right = right
                except Exception as e:
                    logger.error(f"right publish failed: {e}")

            now = time.monotonic()
            if now - last_meta >= meta_period:
                meta = {
                    "available": bool(getattr(cam, "available", False)),
                    "spatial_active": bool(getattr(cam, "spatial_active", False)),
                    "economy": bool(getattr(cam, "economy", False)),
                    "usb_speed": getattr(cam, "usb_speed", None),
                    "depth_fps": float(getattr(cam, "depth_fps", 0.0) or 0.0),
                    "labels": list(getattr(cam, "labels", []) or []),
                    "intrinsics": cam.get_depth_intrinsics(),
                    "imu": cam.get_imu(),
                    "detections": cam.get_spatial_detections(),
                    "heartbeat": now,
                    "child_pid": os.getpid(),
                }
                try:
                    slots["meta"].write_bytes(
                        pickle.dumps(meta, protocol=pickle.HIGHEST_PROTOCOL))
                except Exception as e:
                    logger.error(f"meta publish failed: {e}")
                last_meta = now

            time.sleep(0.002)
    finally:
        logger.info("stopping OakDCamera")
        try:
            cam.cleanup()
        except Exception as e:
            logger.error(f"cleanup failed: {e}")
        for s in slots.values():
            s.close()      # parent owns the blocks and unlinks them
    return 0


if __name__ == "__main__":
    sys.exit(main())
