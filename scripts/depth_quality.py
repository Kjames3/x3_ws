#!/usr/bin/env python3
"""Score OAK depth quality on a STATIC scene, for filter A/B tests.

  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=42 python3 scripts/depth_quality.py --label "filters off"

Reads ~5 s of /oak/rgbd/depth (raw topic; the server's 1/Z correction is
applied here) and reports, over the whole frame and per range band:
  valid   - share of pixels with depth
  noise   - median per-pixel temporal std (mm) over pixels valid in >=80 % of frames
  rel     - noise / depth
  fps     - frames received
Appends to artifacts/depth_quality.ndjson. Keep the robot and scene still.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

LOG = Path(__file__).resolve().parents[1] / 'artifacts' / 'depth_quality.ndjson'
BANDS = [(0.2, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 8.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label', default='')
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--inv-offset', type=float, default=0.0821)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('depth_quality')
    frames = []
    n.create_subscription(Image, '/oak/rgbd/depth/image_raw', lambda m: frames.append(m),
                          qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.05)
    rclpy.shutdown()
    if len(frames) < 10:
        print(f'only {len(frames)} depth frames; is x3_server running with --c1-recording?')
        return
    D = np.stack([np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width)
                  for m in frames]).astype(np.float32) / 1000
    D = np.where(D > 0, D / (1 + a.inv_offset * D), np.nan)
    valid = np.isfinite(D)
    stable = valid.mean(0) >= 0.8
    std = np.nanstd(D, axis=0)
    med = np.nanmedian(D, axis=0)
    out = dict(t=time.time(), label=a.label, frames=len(frames), fps=round(len(frames) / a.seconds, 1),
               valid=round(float(valid.mean()), 3),
               noise_mm=round(float(np.nanmedian(std[stable]) * 1000), 1))
    print(f'{a.label or "(no label)"}: {out["frames"]} frames ({out["fps"]} fps)  '
          f'valid {out["valid"]:.1%}  noise {out["noise_mm"]} mm')
    for lo, hi in BANDS:
        m = stable & (med >= lo) & (med < hi)
        if m.sum() < 200:
            continue
        band = dict(px=int(m.sum()), noise_mm=round(float(np.median(std[m]) * 1000), 1),
                    rel=round(float(np.median(std[m] / med[m])), 4))
        out[f'{lo:g}-{hi:g}m'] = band
        print(f'   {lo:g}-{hi:g} m: {band["px"]:6d} px  noise {band["noise_mm"]:6.1f} mm  ({band["rel"]:.2%} of depth)')
    LOG.parent.mkdir(exist_ok=True)
    with LOG.open('a') as f:
        f.write(json.dumps(out) + '\n')


if __name__ == '__main__':
    main()
