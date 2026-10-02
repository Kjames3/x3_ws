#!/usr/bin/env python3
"""What does a smaller NN input cost the pose model? Offline, on recorded RGB.

  python3 src/c3_input_size_test.py RUN_DIR... --output OUT [--model models/yolo26n-pose.pt]

The OAK runs yolo26n-pose at 640x480 (portrait: 640 high) at ~5.7 packets/s;
inference time on the Myriad scales roughly with input pixels, so a smaller
input is the lever for a faster rate. This runs the same weights with
Ultralytics at several input sizes on every paired frame (the 2026-09-29 A/B
showed Ultralytics matches the OAK blob at full size) and reports, per size:

- found: frames where the yolo11x reference has a person box AND this size
  has a box overlapping it (IoU >= 0.5), split by the reference's depth;
- extra: boxes per 100 frames that overlap no reference box;
- keypoints: share of found people whose shoulders / hips / knees / ankles /
  wrists are visible (confidence >= 0.5).

It does not measure the blob's speed: that needs each size compiled and run on
the camera. Offline only.
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from c1_dataset import load_pair, rgb_on_depth_grid  # noqa: E402

# (height, width), multiples of 32, 4:3 portrait like the camera's 640x480.
SIZES = [(640, 480), (512, 384), (448, 352), (384, 288), (320, 256)]
CONF = 0.5          # the live nn_conf
IOU_MATCH = 0.5
VIS = 0.5
JOINTS = {'shoulders': [5, 6], 'wrists': [9, 10], 'hips': [11, 12], 'knees': [13, 14], 'ankles': [15, 16]}
BINS = [(0.0, 1.8), (1.8, 3.0), (3.0, 4.0)]


def iou(a, b):
    iw = min(a[2], b[2]) - max(a[0], b[0])
    ih = min(a[3], b[3]) - max(a[1], b[1])
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


def depth_bin(z):
    if z is None:
        return 'no depth'
    for lo, hi in BINS:
        if lo <= z < hi:
            return f'{lo:g}-{hi:g} m'
    return 'no depth'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--model', default='models/yolo26n-pose.pt')
    ap.add_argument('--stride', type=int, default=2, help='use every Nth frame (default 2)')
    args = ap.parse_args()
    from ultralytics import YOLO
    model = YOLO(str(ROOT / args.model))
    runs = sorted(r for r in args.runs if (r / 'c3-reference.json').exists())

    # acc[size][bin] -> counters
    acc = {s: defaultdict(lambda: defaultdict(float)) for s in SIZES}
    for run in runs:
        ref = json.loads((run / 'c3-reference.json').read_text())['frames']
        for i in range(0, len(ref), args.stride):
            arrays, row = load_pair(run, i)
            rgb, _ = rgb_on_depth_grid(arrays, row)
            f = ref[i]
            ref_box = max(f['boxes'], key=lambda b: b['conf'])['xyxy'] if f['boxes'] else None
            b = depth_bin(f['person']['z'] if f['person'] else None) if ref_box else None
            for size in SIZES:
                res = model.predict(rgb, imgsz=size, conf=CONF, classes=[0], verbose=False)[0]
                boxes = [[float(v) for v in x.xyxy[0]] for x in res.boxes]
                a = acc[size]
                a['all']['frames'] += 1
                hit = None
                if ref_box is not None:
                    a[b]['ref'] += 1
                    a['all']['ref'] += 1
                    ious = [iou(ref_box, x) for x in boxes]
                    if ious and max(ious) >= IOU_MATCH:
                        hit = int(np.argmax(ious))
                        a[b]['found'] += 1
                        a['all']['found'] += 1
                        kc = res.keypoints.conf[hit].cpu().numpy()
                        for name, idx in JOINTS.items():
                            a[b][name] += float(all(kc[j] >= VIS for j in idx))
                            a['all'][name] += float(all(kc[j] >= VIS for j in idx))
                a['all']['extra'] += sum(1 for k, x in enumerate(boxes)
                                         if k != hit and (ref_box is None or iou(ref_box, x) < IOU_MATCH))
        print(run.name, 'done', flush=True)

    lines = ['| input (h x w) | pixels vs full | where | reference people | found % | extra per 100 frames | '
             + ' | '.join(f'{j} %' for j in JOINTS) + ' |', '|---' * (6 + len(JOINTS)) + '|']
    table = {}
    full_px = SIZES[0][0] * SIZES[0][1]
    for size in SIZES:
        for where in ['all'] + [f'{lo:g}-{hi:g} m' for lo, hi in BINS] + ['no depth']:
            c = acc[size].get(where)
            if not c or not c['ref']:
                continue
            found = c['found']
            row = dict(ref=int(c['ref']), found_pct=100 * found / c['ref'],
                       extra_per_100=100 * acc[size]['all']['extra'] / acc[size]['all']['frames'] if where == 'all' else None,
                       **{j: 100 * c[j] / found if found else 0.0 for j in JOINTS})
            table[f'{size[0]}x{size[1]} | {where}'] = row
            extra = '--' if row['extra_per_100'] is None else f"{row['extra_per_100']:.1f}"
            lines.append(f"| {size[0]}x{size[1]} | {size[0] * size[1] / full_px:.2f} | {where} | {row['ref']} | "
                         f"{row['found_pct']:.1f} | {extra} | "
                         + ' | '.join(f'{row[j]:.0f}' for j in JOINTS) + ' |')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'input_size.json').write_text(json.dumps(
        dict(model=args.model, conf=CONF, stride=args.stride, results=table), indent=1) + '\n')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
