"""Compare Livox voxel and PointCloud2 payload paths with ROS 2 Humble sourced."""

import argparse
from pathlib import Path
import sys
import time

import numpy as np
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'yahboomcar_bringup'))
from yahboomcar_bringup.livox_scan_node import _cloud_data, _first_per_cell, display_voxels


def synthetic_frame(rng):
    xyz = rng.uniform([-4, -4, 0], [4, 4, 1.5], (20_000, 3)).astype('<f4')
    xyz[:10_000, 2] = rng.normal(0, .02, 10_000)   # floor
    xyz[10_000:15_000, 0] = rng.normal(3.5, .02, 5_000)   # wall
    return xyz


def measure(frames):
    names = ('PointCloud2 setter only', 'Build payload + setter',
             'First per cell (5 cm)', 'Display voxelizer',
             'Display voxelizer + build payload + setter')
    samples = {name: {False: [], True: []} for name in names}
    msg = PointCloud2()

    def assign(payload):
        msg.data = payload

    rng = np.random.default_rng(42)
    deadline = time.perf_counter()
    for frame in range(frames + 2):
        xyz = synthetic_frame(rng)
        display = display_voxels(xyz)
        payloads = {fast: _cloud_data(display, fast=fast) for fast in (False, True)}
        # Alternate order to balance cache and frequency effects.
        for fast in (bool(frame % 2), not bool(frame % 2)):
            operations = (
                lambda: assign(payloads[fast]),
                lambda: assign(_cloud_data(display, fast=fast)),
                lambda: _first_per_cell(xyz, .05, fast=fast),
                lambda: display_voxels(xyz, fast=fast),
                lambda: assign(_cloud_data(display_voxels(xyz, fast=fast), fast=fast)),
            )
            for name, operation in zip(names, operations):
                start = time.perf_counter_ns()
                operation()
                elapsed_us = (time.perf_counter_ns() - start) / 1e3
                if frame >= 2:
                    samples[name][fast].append(elapsed_us)
        deadline += .1
        time.sleep(max(0, deadline - time.perf_counter()))
    return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, default=60)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error('use at least one measured frame')
    print(f'{args.frames} frames per path, 20,000 input points/frame at 10 Hz')
    print(f'{"Operation":46s} {"Old p50":>10s} {"Fast p50":>10s} {"Old p95":>10s} {"Fast p95":>10s} (us/frame)')
    for name, paths in measure(args.frames).items():
        old, old_p95 = np.percentile(paths[False], [50, 95])
        fast, fast_p95 = np.percentile(paths[True], [50, 95])
        print(f'{name:46s} {old:10.2f} {fast:10.2f} {old_p95:10.2f} {fast_p95:10.2f}')


if __name__ == '__main__':
    main()
