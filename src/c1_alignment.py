#!/usr/bin/env python3
"""Measure annotated rectangular target edges in nominally registered RGB-D.

The ROI is chosen on rectified RGB, independently of the depth boundary.
Invalid depth is an uncertainty band, never background or a zero-error match.
This diagnostic does not modify calibration or certify the entire field of view.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def boundary_interval(depth_row, expected_x, target_mm, side, radius=30):
    """Return a transition interval bracketed by actual foreground/background.

    Foreground is within 100 mm of measured target distance; background must
    exceed it by 150 mm. Ambiguous/multiple transitions are unscored.
    """
    low, high = max(0, expected_x-radius), min(len(depth_row), expected_x+radius+1)
    xs = np.arange(low, high)
    d = depth_row[low:high].astype(float)
    foreground = (d > 0) & (np.abs(d-target_mm) <= 100)
    background = d > target_mm+150
    labels = np.where(foreground, 1, np.where(background, 2, 0))
    good = np.flatnonzero(labels)
    wanted = (2, 1) if side == 'left' else (1, 2)
    candidates = [(int(xs[a]), int(xs[b])) for a, b in zip(good, good[1:])
                  if (labels[a], labels[b]) == wanted]
    if len(candidates) != 1:
        return None
    a, b = candidates[0]
    # Pixel centers constrain the transition between classified observations.
    return float(a), float(b)


def measure(rgb, depth, support, roi, target_mm, stride=4):
    """RGB gradients refine each independently selected edge within ±5 pixels."""
    import cv2
    x0, y0, x1, y1 = roi
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(float)
    rows=[]
    for y in range(y0, y1, stride):
        for side, x in [('left', x0), ('right', x1)]:
            lo, hi = max(1, x-5), min(gray.shape[1]-2, x+5)
            if hi <= lo or not support[y, lo:hi+2].all():
                rows.append(dict(y=y, side=side, status='unsupported_rgb')); continue
            # Adjacent-pixel difference places the edge at a half-pixel coordinate.
            gradients = np.abs(gray[y,lo+1:hi+2]-gray[y,lo:hi+1])
            index = int(np.argmax(gradients))
            if gradients[index] < 15:
                rows.append(dict(y=y, side=side, status='weak_rgb_edge')); continue
            rgb_x = lo+index+.5
            interval = boundary_interval(depth[y], int(round(rgb_x)), target_mm, side)
            if interval is None:
                rows.append(dict(y=y, side=side, status='no_unique_depth_transition')); continue
            a,b = interval
            signed = (a+b)/2-rgb_x
            lower = max(a-rgb_x, rgb_x-b, 0)
            upper = max(abs(a-rgb_x), abs(b-rgb_x))
            rows.append(dict(y=y,side=side,status='measured',rgb_edge_coordinate_px=rgb_x,
                             depth_interval_px=[a,b],signed_midpoint_error_px=signed,
                             absolute_error_lower_px=lower, absolute_error_upper_px=upper,
                             uncertainty_width_px=b-a))
    return rows


def measure_rectangle(rgb, depth, support, roi, target_mm):
    vertical = [dict(r, axis='x') for r in measure(rgb, depth, support, roi, target_mm)]
    x0,y0,x1,y1 = roi
    horizontal = measure(np.transpose(rgb,(1,0,2)), depth.T, support.T,
                         (y0,x0,y1,x1), target_mm)
    horizontal = [dict(r, axis='y', side='top' if r['side']=='left' else 'bottom')
                  for r in horizontal]
    return vertical + horizontal


def summarize(rows):
    from collections import Counter
    measured=[r for r in rows if r['status']=='measured']
    out=dict(samples=len(rows),status_counts=dict(Counter(r['status'] for r in rows)),
             measured_fraction=len(measured)/len(rows) if rows else 0)
    for key in ['signed_midpoint_error_px','absolute_error_lower_px',
                'absolute_error_upper_px','uncertainty_width_px']:
        values=[r[key] for r in measured]
        out[key] = dict(median=float(np.median(values)),p95=float(np.percentile(values,95))) if values else None
    # Provisional local engineering gate, fixed before the target capture.
    out['local_edge_gate_pass'] = bool(measured and out['measured_fraction'] >= .8
        and out['absolute_error_upper_px']['p95'] <= 5)
    out['scope']='annotated target edges only; not full camera calibration'
    return out


def main():
    from c1_dataset import load_pair, rgb_on_depth_grid, write_json
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset', type=Path)
    p.add_argument('--roi',type=int,nargs=4,required=True,metavar=('LEFT','TOP','RIGHT','BOTTOM'))
    p.add_argument('--distance-mm',type=float,required=True)
    p.add_argument('--every',type=int,default=6)
    args=p.parse_args()
    if args.every<1 or args.distance_mm<=0: p.error('positive distance/every required')
    entries=json.loads((args.dataset/'pair-index.json').read_text())
    rows=[]; references=[]
    for i in range(0,len(entries),args.every):
        arrays,row=load_pair(args.dataset,i)
        rgb,support=rgb_on_depth_grid(arrays,row)
        x0,y0,x1,y1=args.roi
        h,w=arrays['depth'].shape
        if not 0<=x0<x1<w or not 0<=y0<y1<h: p.error('ROI outside image')
        samples=measure_rectangle(rgb,arrays['depth'],support,args.roi,args.distance_mm)
        rows.extend(dict(r,pair_index=i) for r in samples)
        references.append(dict(index=i,identity=row['identity'],rgb_sha256=row['rgb']['sha256'],
                               depth_sha256=row['depth']['sha256']))
    report=dict(schema='x3.c1.alignment.v1',roi=args.roi,target_distance_mm=args.distance_mm,
                configuration=dict(search_radius_px=30,rgb_refinement_radius_px=5,
                    foreground_tolerance_mm=100,background_clearance_mm=150,
                    required_measured_fraction=.8,max_p95_error_upper_px=5),
                summary=summarize(rows),
                per_edge={side:summarize([r for r in rows if r['side']==side])
                          for side in ['left','right','top','bottom']},
                references=references,samples=rows)
    write_json(args.dataset/'alignment.json',report)
    print(json.dumps(report['summary'],indent=2))
    return 0 if all(v['local_edge_gate_pass'] for v in report['per_edge'].values()) else 2


if __name__=='__main__':raise SystemExit(main())
