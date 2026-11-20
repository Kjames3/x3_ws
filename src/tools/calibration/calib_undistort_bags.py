#!/usr/bin/env python3
"""Rewrite calib_capture.py bags with undistorted, zero-distortion images.

direct_visual_lidar_calibration only reads plumb_bob/fisheye camera_info, and the
OAK EEPROM model has 14 coefficients. This undistorts each /image with the full
model (cv2 accepts all 14), and writes a new bag whose /camera_info is plumb_bob
with D = 0 and the new K. The /points message is copied unchanged.

    source /opt/ros/humble/setup.bash
    python3 calib_undistort_bags.py ~/calib/2026-11-20 --camera front --out ~/calib/2026-11-20_rect

Bags with K all zero (the uncalibrated Arducam) are skipped; pass --intrinsics
JSON {K, D, w, h, model: fisheye} to use an external calibration instead.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

TYPES = {"/image": Image, "/camera_info": CameraInfo, "/points": PointCloud2}


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    msgs = {}
    while r.has_next():
        topic, data, t = r.read_next()
        msgs[topic] = (deserialize_message(data, TYPES[topic]), t)
    return msgs


def write_bag(path, msgs):
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, (msg, t) in msgs.items():
        w.create_topic(rosbag2_py.TopicMetadata(
            name=topic, type=f"sensor_msgs/msg/{type(msg).__name__}",
            serialization_format="cdr", offered_qos_profiles=""))
    for topic, (msg, t) in msgs.items():
        w.write(topic, serialize_message(msg), t)
    del w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("--camera", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--intrinsics", help="override JSON {K, D, w, h, model}")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    override = json.loads(Path(a.intrinsics).read_text()) if a.intrinsics else None
    newK = None
    for bag in sorted(Path(a.src).expanduser().glob(f"s*_{a.camera}")):
        dst = out / bag.name
        if dst.exists():
            print(f"{dst} exists, skipped")
            continue
        intr = override or json.loads((bag / "intrinsics.json").read_text())
        K = np.array(intr["K"], float).reshape(3, 3)
        D = np.array(intr["D"], float)
        if not K.any():
            print(f"{bag.name}: no intrinsics, skipped")
            continue
        msgs = read_bag(bag)
        img_msg = msgs["/image"][0]
        ch = 3 if img_msg.encoding == "bgr8" else 1
        img = np.frombuffer(bytes(img_msg.data), np.uint8).reshape(img_msg.height, img_msg.width, ch).squeeze()
        h, w = img.shape[:2]
        fisheye = str(intr.get("model", "")).lower() in ("fisheye", "equidistant")
        if newK is None:
            if fisheye:
                newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    K, D[:4], (w, h), np.eye(3), balance=0.0)
            else:
                newK = K.copy()  # getOptimalNewCameraMatrix explodes on rational fits that extrapolate badly
            print(f"new K:\n{newK}")
        if fisheye:
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D[:4], np.eye(3), newK, (w, h), cv2.CV_16SC2)
        else:
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_16SC2)
        rect = np.ascontiguousarray(cv2.remap(img, m1, m2, cv2.INTER_LINEAR))
        img_msg.data = rect.tobytes()
        img_msg.step = int(rect.strides[0])
        info = msgs["/camera_info"][0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(v) for v in newK.ravel()]
        info.p = [newK[0, 0], 0.0, newK[0, 2], 0.0, 0.0, newK[1, 1], newK[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        write_bag(dst, msgs)
        cv2.imwrite(str(dst / "image_rect.png"), rect)
        (dst / "intrinsics_rect.json").write_text(json.dumps(
            {"K": newK.ravel().tolist(), "D": [0.0] * 5, "w": w, "h": h, "model": "plumb_bob"}, indent=2))
        print(f"{bag.name} -> {dst}")


if __name__ == "__main__":
    main()
