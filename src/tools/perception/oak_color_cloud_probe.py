#!/usr/bin/env python3
"""Offline-only colored-cloud probe on a rectified RGB/depth NPZ sample.

RGB input is BGR on the depth grid; depth is raw millimetres. This intentionally
has no live camera, ROS, navigation, or GUI integration.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


from oak_color_geometry import prepare_geometry


def project(rgb, depth, support, k, correction, budget=6000, edge_margin=12):
    if budget < 1:
        raise ValueError("budget must be positive")
    z, valid, safe = prepare_geometry(rgb, depth, support, correction, edge_margin)
    y, x = np.nonzero(safe)
    if len(x) > budget:
        indices = np.linspace(0, len(x) - 1, budget, dtype=int)
        y, x = y[indices], x[indices]
    zz = z[y, x]
    xyz = np.column_stack(((x-k[0, 2])*zz/k[0, 0], (y-k[1, 2])*zz/k[1, 1], zz)).astype('<f4')
    colors = rgb[y, x, ::-1].copy()
    return xyz, colors, safe, dict(valid_pixels=int(valid.sum()),
        retained_pixels=int(safe.sum()), points=len(x), edge_margin_px=edge_margin,
        note='Experimental interior mask; full-image alignment remains unqualified')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sample', type=Path)
    p.add_argument('--metadata', type=Path, required=True, help='ROS echo metadata YAML')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--edge-margin', type=int, default=12)
    a = p.parse_args()
    import yaml
    meta = json.loads(next(yaml.safe_load_all(a.metadata.read_text()))['data'])
    c = meta['calibration']
    if c['depth_recorded'] != 'raw_uncorrected':
        raise ValueError('probe expects uncorrected input; refusing double correction')
    sample = np.load(a.sample)
    if sample['depth'].shape != (c['height'], c['width']):
        raise ValueError('sample/calibration dimensions disagree')
    start = time.perf_counter()
    xyz, rgb, mask, report = project(sample['rgb'],sample['depth'],sample['support'],
        np.asarray(c['depth_k']).reshape(3,3),c['depth_inv_offset_per_m'],edge_margin=a.edge_margin)
    report['offline_work_ms'] = (time.perf_counter()-start)*1000
    a.output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(a.output/'cloud.npz', xyz=xyz, rgb=rgb, valid=mask)
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
