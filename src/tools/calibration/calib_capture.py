#!/usr/bin/env python3
"""Station-by-station lidar-to-camera calibration capture into ROS2 bags.

Runs ON THE ROBOT with x3_server STOPPED (this tool owns every sensor). Push
the robot by hand to a station, keep it still, press Enter; the tool records
one static capture: it accumulates Livox clouds for --seconds, grabs one frame
from each camera at the capture midpoint, and writes one rosbag2 (sqlite3)
per camera per station:

    <out>/<station>_<camera>/   topics /image /camera_info /points
        image.png  meta.json  intrinsics.json

These bags feed koide3 direct_visual_lidar_calibration (`preprocess`), one
camera at a time. Nothing is published to ROS; no /cmd_vel, no serial devices.

Start the Livox driver yourself in another terminal first (PointCloud2 out):

    source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash; ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=$HOME/ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json -p cmdline_input_bd_code:=livox0000000001

Then, in a shell with ROS2 Humble sourced:

    python3 src/tools/calibration/calib_capture.py --out ~/calib/2026-11-20 \\
        [--cameras front,rear,right] [--seconds 12] [--station-name NAME]

Prompt: Enter = capture next station (s01, s02, ...), 'q' = quit. Any other
text typed at the prompt is used as that station's name.

Cameras: front/rear = OAK by device id (depthai v3, full-FOV output),
right = Arducam OV9281 UVC, left = Innomaker OV9281 UVC (mono8, rotated
180 deg); both need fisheye intrinsics before they can be solved.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import glob
import json
import os
import re
import threading
import time

import cv2
import numpy as np

LIVOX_CMD = (
    "source /opt/ros/humble/setup.bash; source ~/ws_livox/install/setup.bash; "
    "ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args "
    "-r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 "
    "-p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 "
    "-p frame_id:=livox_frame -p user_config_path:=$HOME/ws_livox/install/"
    "livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json "
    "-p cmdline_input_bd_code:=livox0000000001"
)

CAMERAS = {
    "front": {"kind": "oak", "id": "14442C10B1F4D2D600", "frame_id": "oak_rgb_camera_optical_frame"},
    "rear": {"kind": "oak", "id": "14442C103181D7D600", "frame_id": "oak_rear_rgb_camera_optical_frame"},
    "right": {"kind": "uvc", "id": "/dev/v4l/by-id/*Arducam*-video-index0",
              "frame_id": "arducam_optical_frame", "rotate": 0},
    # Innomaker U20CAM-9281M, added 2026-11-21; mounted upside down, so frames are
    # rotated 180 deg to match the image x3_server publishes.
    "left": {"kind": "uvc", "id": "/dev/v4l/by-id/*SN0001-video-index0",
             "frame_id": "left_cam_optical_frame", "rotate": 180},
}
CLOUD_TOPIC = "/livox/lidar"
FIRST_CLOUD_TIMEOUT = 3.0
MOVED_THRESHOLD = 6.0
MAX_FRAME_AGE = 2.0


# ---------------------------------------------------------------------------
# Cameras: each runs a grabber thread that keeps the latest frame.
# ---------------------------------------------------------------------------

class CameraBase:
    def __init__(self, name):
        self.name = name
        self.frame_id = CAMERAS[name]["frame_id"]
        self.device_id = CAMERAS[name]["id"]
        self.encoding = "bgr8"
        self.w = self.h = 0
        self.K = [0.0] * 9
        self.D = []
        self.model = ""
        self.ready = threading.Event()
        self.ok = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame = None
        self._frame_t = 0.0
        self._thread = threading.Thread(target=self._run_guarded, daemon=True)

    def start(self, timeout=20.0):
        self._thread.start()
        self.ready.wait(timeout)
        return self.ok

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5)

    def _set_frame(self, frame):
        with self._lock:
            self._frame = frame
            self._frame_t = time.time()

    def latest(self):
        """Return a copy of the newest frame, or None if missing/stale."""
        with self._lock:
            if self._frame is None or time.time() - self._frame_t > MAX_FRAME_AGE:
                return None
            return self._frame.copy()

    def _run_guarded(self):
        try:
            self._run()
        except Exception as exc:
            print(f"[{self.name}] camera failed: {exc}")
        finally:
            self.ok = False if self._frame is None else self.ok
            self.ready.set()


class OakCamera(CameraBase):
    def _run(self):
        import depthai as dai
        info = dai.DeviceInfo(self.device_id)
        with dai.Device(info) as dev:
            socket = dai.CameraBoardSocket.CAM_A
            # Native sensor size -> largest same-aspect size <= 1920 wide (no crop).
            nat_w, nat_h = None, None
            for feat in dev.getConnectedCameraFeatures():
                if feat.socket == socket:
                    nat_w, nat_h = int(feat.width), int(feat.height)
                    break
            if not nat_w:
                raise RuntimeError("CAM_A not found in getConnectedCameraFeatures()")
            if nat_w <= 1920:
                w, h = nat_w, nat_h
            else:
                w = 1920
                h = int(round(1920.0 * nat_h / nat_w / 2.0)) * 2
            self.w, self.h = w, h
            print(f"[{self.name}] sensor native {nat_w}x{nat_h} -> requesting {w}x{h}")

            calib = dev.readCalibration()
            self.K = [float(v) for row in calib.getCameraIntrinsics(socket, w, h) for v in row]
            self.D = [float(v) for v in calib.getDistortionCoefficients(socket)]
            try:
                self.model = str(calib.getDistortionModel(socket))
            except Exception as exc:
                self.model = f"unknown ({exc})"
            print(f"[{self.name}] distortion model: {self.model}  ({len(self.D)} coeffs)")

            with dai.Pipeline(dev) as pipeline:
                cam = pipeline.create(dai.node.Camera).build(socket)
                try:
                    out = cam.requestOutput((w, h), type=dai.ImgFrame.Type.BGR888i, fps=5)
                except Exception as exc:
                    print(f"[{self.name}] BGR888i rejected ({exc}); falling back to NV12")
                    out = cam.requestOutput((w, h), type=dai.ImgFrame.Type.NV12, fps=5)
                queue = out.createOutputQueue(maxSize=1, blocking=False)
                pipeline.start()
                while not self._stop.is_set():
                    pkt = queue.tryGet()
                    if pkt is None:
                        time.sleep(0.01)
                        continue
                    frame = pkt.getCvFrame()
                    if frame.ndim == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    if self._frame is None:
                        self.h, self.w = frame.shape[:2]
                        self.ok = True
                        self.ready.set()
                    self._set_frame(frame)


class UvcCamera(CameraBase):
    def __init__(self, name):
        super().__init__(name)
        self.encoding = "mono8"
        self.model = "equidistant"
        self.D = []

    def _run(self):
        pat = CAMERAS[self.name]["id"]
        paths = sorted(glob.glob(pat))
        if not paths:
            raise RuntimeError(f"no {pat}")
        self.device_id = paths[0]
        cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError(f"cannot open {self.device_id}")
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            # Reading continuously in this thread keeps the V4L buffer drained,
            # so latest() is always the newest frame.
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    continue
                if frame.ndim == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if CAMERAS[self.name].get("rotate") == 180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                if self._frame is None:
                    self.h, self.w = frame.shape[:2]
                    print(f"[{self.name}] opened {self.device_id} at {self.w}x{self.h}")
                    self.ok = True
                    self.ready.set()
                self._set_frame(frame)
        finally:
            cap.release()


# ---------------------------------------------------------------------------
# Lidar: rclpy subscriber collecting clouds while a capture is active.
# ---------------------------------------------------------------------------

class CloudCollector:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import PointCloud2

        rclpy.init()
        self._rclpy = rclpy
        self.node = Node("calib_capture")
        self.node.create_subscription(PointCloud2, CLOUD_TOPIC, self._cb, qos_profile_sensor_data)
        self._lock = threading.Lock()
        self._active = False
        self._clouds = []
        self._frame_id = ""
        self._first = threading.Event()
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def _cb(self, msg):
        with self._lock:
            if not self._active:
                return
            arr = self._to_xyzi(msg)
            if arr is None:
                return
            self._frame_id = msg.header.frame_id
            self._clouds.append(arr)
        self._first.set()

    @staticmethod
    def _to_xyzi(msg):
        """Parse a PointCloud2 into an (N,4) float32 array, dropping NaN/zero points."""
        names = {f.name: f for f in msg.fields}
        if not all(k in names for k in ("x", "y", "z")):
            return None
        n = msg.width * msg.height
        if n == 0:
            return None
        np_types = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
                    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}
        dt_names, dt_formats, dt_offsets = [], [], []
        for key in ("x", "y", "z", "intensity"):
            if key in names:
                dt_names.append(key)
                dt_formats.append(np_types[names[key].datatype])
                dt_offsets.append(names[key].offset)
        dtype = np.dtype({"names": dt_names, "formats": dt_formats, "offsets": dt_offsets,
                          "itemsize": msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        out = np.zeros((n, 4), dtype=np.float32)
        out[:, 0] = raw["x"]
        out[:, 1] = raw["y"]
        out[:, 2] = raw["z"]
        if "intensity" in raw.dtype.names:
            out[:, 3] = raw["intensity"]
        xyz = out[:, :3]
        keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0.0)
        return out[keep]

    def begin(self):
        with self._lock:
            self._clouds = []
            self._frame_id = ""
            self._first.clear()
            self._active = True

    def wait_first(self, timeout):
        return self._first.wait(timeout)

    def end(self):
        with self._lock:
            self._active = False
            clouds, frame_id = self._clouds, self._frame_id
            self._clouds = []
        return clouds, frame_id

    def shutdown(self):
        try:
            self._exec.shutdown()
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Bag writing
# ---------------------------------------------------------------------------

def build_cloud_msg(points, frame_id, stamp_ns):
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z", "intensity"))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


def build_image_msg(frame, encoding, frame_id, stamp_ns):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(frame.strides[0])
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def build_info_msg(cam, w, h, frame_id, stamp_ns):
    from sensor_msgs.msg import CameraInfo
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.width, msg.height = int(w), int(h)
    K = list(cam.K)
    msg.k = K
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [K[0], K[1], K[2], 0.0, K[3], K[4], K[5], 0.0, K[6], K[7], K[8], 0.0]
    if cam.kind_is_oak:
        msg.distortion_model = "rational_polynomial"
        msg.d = [float(v) for v in cam.D[:8]]
    else:
        msg.distortion_model = "equidistant"
        msg.d = []
    return msg


def write_bag(bag_dir, image_msg, info_msg, cloud_msg, stamp_ns):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name, typ in (("/image", "sensor_msgs/msg/Image"),
                      ("/camera_info", "sensor_msgs/msg/CameraInfo"),
                      ("/points", "sensor_msgs/msg/PointCloud2")):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=typ, serialization_format="cdr", offered_qos_profiles=""))
    writer.write("/image", serialize_message(image_msg), stamp_ns)
    writer.write("/camera_info", serialize_message(info_msg), stamp_ns)
    writer.write("/points", serialize_message(cloud_msg), stamp_ns)
    del writer  # closes and flushes the sqlite file


def gray_small(frame):
    g = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.resize(g, (160, 100), interpolation=cv2.INTER_AREA).astype(np.float32)


# ---------------------------------------------------------------------------
# Station capture
# ---------------------------------------------------------------------------

def capture_station(station, cams, collector, out_dir, seconds):
    t0_ns = time.time_ns()
    first = {n: c.latest() for n, c in cams.items()}
    collector.begin()
    if not collector.wait_first(FIRST_CLOUD_TIMEOUT):
        collector.end()
        print(f"ERROR: no cloud on {CLOUD_TOPIC} within {FIRST_CLOUD_TIMEOUT:.0f} s; "
              f"is the Livox driver running? Station {station} aborted.")
        return False

    print(f"  accumulating {seconds:.0f} s, keep the robot still ...")
    time.sleep(seconds / 2.0)
    mid = {n: c.latest() for n, c in cams.items()}
    time.sleep(seconds / 2.0)
    last = {n: c.latest() for n, c in cams.items()}
    clouds, frame_id = collector.end()
    if not clouds:
        print(f"ERROR: no usable points collected; station {station} aborted.")
        return False
    points = np.concatenate(clouds, axis=0)
    cloud_msg = build_cloud_msg(points, frame_id, t0_ns)
    print(f"  {len(clouds)} clouds, {points.shape[0]} points, frame '{frame_id}'")

    for name, cam in cams.items():
        frame = mid[name]
        if frame is None:
            print(f"  [{name}] no fresh frame at midpoint; skipped")
            continue
        if first[name] is not None and last[name] is not None:
            diff = float(np.mean(np.abs(gray_small(first[name]) - gray_small(last[name]))))
            if diff > MOVED_THRESHOLD:
                print(f"  [{name}] MOVED? redo station (frame diff {diff:.1f} > {MOVED_THRESHOLD:.0f})")
            else:
                print(f"  [{name}] still (frame diff {diff:.1f})")
        else:
            print(f"  [{name}] stillness check skipped (missing first/last frame)")

        h, w = frame.shape[:2]
        bag_dir = out_dir / f"{station}_{name}"
        if bag_dir.exists():
            print(f"  [{name}] {bag_dir} already exists; skipped")
            continue
        image_msg = build_image_msg(frame, cam.encoding, cam.frame_id, t0_ns)
        info_msg = build_info_msg(cam, w, h, cam.frame_id, t0_ns)
        write_bag(bag_dir, image_msg, info_msg, cloud_msg, t0_ns)
        cv2.imwrite(str(bag_dir / "image.png"), frame)
        intr = {"camera": name, "model": cam.model, "w": w, "h": h, "K": cam.K, "D": cam.D}
        (bag_dir / "intrinsics.json").write_text(json.dumps(intr, indent=2))
        meta = {"camera": name, "device_id": cam.device_id, "station": station,
                "w": w, "h": h, "K": cam.K, "D": cam.D, "model": cam.model,
                "encoding": cam.encoding, "frame_id": cam.frame_id,
                "cloud_frame_id": frame_id, "n_points": int(points.shape[0]),
                "n_clouds": len(clouds), "seconds": seconds, "stamp_ns": t0_ns}
        (bag_dir / "meta.json").write_text(json.dumps(meta, indent=2))
        print(f"  [{name}] wrote {bag_dir}")
    return True


def next_default_station(out_dir):
    used = set()
    for p in out_dir.glob("s[0-9][0-9]*_*"):
        m = re.match(r"s(\d+)_", p.name)
        if m:
            used.add(int(m.group(1)))
    i = 1
    while i in used:
        i += 1
    return f"s{i:02d}"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True, help="output directory for the bags")
    ap.add_argument("--cameras", default="front,rear,right")
    ap.add_argument("--seconds", type=float, default=12.0, help="cloud accumulation time")
    ap.add_argument("--station-name", default=None, help="name for the first station")
    args = ap.parse_args()

    names = [n.strip() for n in args.cameras.split(",") if n.strip()]
    bad = [n for n in names if n not in CAMERAS]
    if bad:
        ap.error(f"unknown camera(s) {bad}; choose from {sorted(CAMERAS)}")
    out_dir = _Path(os.path.expanduser(args.out)).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Start the Livox driver in another terminal first:\n")
    print(f"  {LIVOX_CMD}\n")

    cams = {}
    for n in names:
        cam = UvcCamera(n) if CAMERAS[n]["kind"] == "uvc" else OakCamera(n)
        cam.kind_is_oak = CAMERAS[n]["kind"] == "oak"
        cams[n] = cam
    live = {}
    for n, cam in cams.items():
        print(f"[{n}] opening ...")
        if cam.start():
            live[n] = cam
            if cam.kind_is_oak:
                (out_dir / f"intrinsics_{n}.json").write_text(json.dumps(
                    {"camera": n, "model": cam.model, "w": cam.w, "h": cam.h,
                     "K": cam.K, "D": cam.D}, indent=2))
        else:
            print(f"[{n}] WARNING: could not open; continuing without it")
            cam.stop()
    if not live:
        print("No cameras opened; nothing to capture.")
        return 1

    collector = CloudCollector()
    pending_name = args.station_name
    try:
        while True:
            station = pending_name or next_default_station(out_dir)
            pending_name = None
            try:
                ans = input(f"\n[{station}] Position the robot, let it settle, "
                            f"Enter = capture, 'q' = quit, or type a station name: ").strip()
            except EOFError:
                break
            if ans.lower() == "q":
                break
            if ans:
                station = ans
            capture_station(station, live, collector, out_dir, args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        collector.shutdown()
        for cam in live.values():
            cam.stop()
    return 0


if __name__ == "__main__":
    _sys.exit(main())
