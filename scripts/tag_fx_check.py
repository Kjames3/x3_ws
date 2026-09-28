#!/usr/bin/env python3
"""Measure the OAK CAM_A focal length with a known AprilTag (36h11).

  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=42 python3 scripts/tag_fx_check.py [--size 0.144] [--id 1] [--label NAME]

Grabs ~3 s of /oak/rgbd/rgb (the NN preview, 480x640) with the paired
/oak/rgbd/depth, finds the tag, and prints per frame-set:
  - tag centre column u / row v, side lengths in pixels;
  - tag depth Z (median of the aligned depth inside the tag);
  - fx = horizontal size * Z / size, fy likewise (independent of the
    calibration being tested), and the lateral offset the CURRENT fx=677
    would report.
Each run is appended to artifacts/tag_fx_check.ndjson with --label, so
shifted placements (0, 0.3, 0.6 m sideways) can be compared afterwards:
fx = (u1 - u2) * Z / (x2 - x1) gives it again from the shift alone.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

LOG = Path(__file__).resolve().parents[1] / 'artifacts' / 'tag_fx_check.ndjson'


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=float, default=0.144, help='tag black-square side, m')
    ap.add_argument('--id', type=int, default=1)
    ap.add_argument('--label', default='')
    ap.add_argument('--seconds', type=float, default=3.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('tag_fx_check')
    buf = {'rgb': [], 'depth': [], 'k': None}
    node.create_subscription(Image, '/oak/rgbd/rgb/image_raw', lambda m: buf['rgb'].append(m),
                             qos_profile_sensor_data)
    node.create_subscription(Image, '/oak/rgbd/depth/image_raw', lambda m: buf['depth'].append(m),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/oak/rgbd/rgb/camera_info',
                             lambda m: buf.__setitem__('k', m.k), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < a.seconds or buf['k'] is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 15:
            break
    rclpy.shutdown()
    if not buf['rgb'] or not buf['depth']:
        print('No /oak/rgbd frames: is x3_server running with --c1-recording?')
        return
    fx0, cx0, fy0, cy0 = buf['k'][0], buf['k'][2], buf['k'][4], buf['k'][5]

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                  cv2.aruco.DetectorParameters())
    rows = []
    for m in buf['rgb']:
        d = min(buf['depth'], key=lambda x: abs(stamp(x) - stamp(m)))
        if abs(stamp(d) - stamp(m)) > 0.02:
            continue
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None or a.id not in ids.ravel():
            continue
        c = corners[list(ids.ravel()).index(a.id)][0]   # TL, TR, BR, BL
        cv2.cornerSubPix(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), c, (3, 3), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        top, bottom = np.linalg.norm(c[1] - c[0]), np.linalg.norm(c[2] - c[3])
        left, right = np.linalg.norm(c[3] - c[0]), np.linalg.norm(c[2] - c[1])
        D = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.width).astype(np.float32) / 1000
        mask = np.zeros(D.shape, np.uint8)
        cv2.fillConvexPoly(mask, c.astype(np.int32), 1)
        z = D[(mask > 0) & (D > 0.2)]
        if z.size < 30:
            continue
        rows.append(dict(u=float(c[:, 0].mean()), v=float(c[:, 1].mean()),
                         w=float((top + bottom) / 2), h=float((left + right) / 2), z=float(np.median(z))))
        last = img.copy()
        cv2.polylines(last, [c.astype(np.int32)], True, (0, 0, 255), 2)
    if not rows:
        print(f'Tag id {a.id} not found in {len(buf["rgb"])} frames. Is it fully in view and lit?')
        return
    cv2.imwrite('/tmp/tag_fx_check.jpg', last)
    R = {k: float(np.median([r[k] for r in rows])) for k in rows[0]}
    fx = R['w'] * R['z'] / a.size
    fy = R['h'] * R['z'] / a.size
    lateral_now = -(R['u'] - cx0) * R['z'] / fx0
    lateral_meas = -(R['u'] - cx0) * R['z'] / fx
    print(f'{len(rows)} frames with the tag (of {len(buf["rgb"])}); calibration in use fx={fx0:.1f} fy={fy0:.1f} cx={cx0:.1f}')
    print(f'tag centre u={R["u"]:.1f} v={R["v"]:.1f}  size {R["w"]:.1f} x {R["h"]:.1f} px  depth Z={R["z"]:.3f} m')
    print(f'==> measured fx = {fx:.1f}   fy = {fy:.1f}   (ratio to the calibration {fx / fx0:.3f}, {fy / fy0:.3f})')
    print(f'    lateral offset: {lateral_now:+.3f} m with fx={fx0:.0f}, {lateral_meas:+.3f} m with measured fx (+ = left)')
    LOG.parent.mkdir(exist_ok=True)
    with LOG.open('a') as f:
        f.write(json.dumps(dict(t=time.time(), label=a.label, size=a.size, fx_cal=fx0, cx_cal=cx0,
                                fy_cal=fy0, n=len(rows), **R, fx=fx, fy=fy)) + '\n')
    print(f'appended to {LOG}; annotated frame /tmp/tag_fx_check.jpg')


if __name__ == '__main__':
    main()
