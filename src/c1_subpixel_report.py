#!/usr/bin/env python3
"""Stationary subpixel trial analysis; separate correspondence coverage from error.

Does not replace the original alignment gate or fit calibration/range offsets.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from c1_dataset import load_pair, rgb_on_depth_grid, write_json
from c1_alignment import measure_rectangle, summarize


def distribution(values):
    a=np.asarray(values,dtype=float)
    a=a[np.isfinite(a)]
    if not len(a):return None
    return dict(n=int(len(a)),mean=float(a.mean()),sd=float(a.std()),
                p05=float(np.percentile(a,5)),median=float(np.median(a)),
                p95=float(np.percentile(a,95)),min=float(a.min()),max=float(a.max()))


def separate_edge_evidence(rows):
    result=summarize(rows)
    measured=[r for r in rows if r['status']=='measured']
    narrow=[r for r in measured if r['uncertainty_width_px']<=2]
    result['absolute_midpoint_error_px_measured_only']=distribution([
        abs(r['signed_midpoint_error_px']) for r in measured])
    result['narrow_interval_diagnostic']=dict(max_interval_width_px=2,
        samples=len(narrow),fraction_of_all=len(narrow)/len(rows) if rows else 0,
        absolute_midpoint_error_px=distribution([abs(r['signed_midpoint_error_px']) for r in narrow]),
        note='Conditional diagnostic, not a replacement pass gate; omitted rows remain unqualified.')
    return result


def range_evidence(patches, reference_mm):
    frames=[];medians=[];valid_fractions=[]
    for i,patch in enumerate(patches):
        valid=patch[patch>0]
        fraction=len(valid)/patch.size
        median=float(np.median(valid)) if len(valid) else None
        frames.append(dict(pair_index=i,valid_fraction=fraction,median_mm=median,
                           error_mm=median-reference_mm if median is not None else None))
        valid_fractions.append(fraction)
        if median is not None:medians.append(median)
    return dict(reference_mm=reference_mm,frame_median_mm=distribution(medians),
        frame_median_error_mm=distribution([m-reference_mm for m in medians]),
        frame_valid_fraction=distribution(valid_fractions),frames=frames,
        note='Fixed interior ROI. Frame-median variation is not absolute accuracy or per-pixel noise.')


def analyze(directory, roi, patch_roi, reference_mm, declared_mode):
    entries=json.loads((directory/'pair-index.json').read_text())
    patches=[];rows=[];modes=Counter();calibrations=[];rgb_first=rgb_last=None
    for i in range(len(entries)):
        arrays,row=load_pair(directory,i)
        h,w=arrays['depth'].shape
        for box in (roi,patch_roi):
            x0,y0,x1,y1=box
            if not 0<=x0<x1<w or not 0<=y0<y1<h:raise ValueError('ROI outside frame')
        x0,y0,x1,y1=patch_roi
        patches.append(arrays['depth'][y0:y1,x0:x1].copy())
        cal=row['metadata']['calibration']
        modes[str(cal.get('stereo_subpixel','not_recorded'))]+=1
        if not calibrations or cal!=calibrations[-1]:calibrations.append(cal)
        if i%6==0 or i==len(entries)-1:
            rgb,support=rgb_on_depth_grid(arrays,row)
            if rgb_first is None:rgb_first=rgb.copy()
            rgb_last=rgb.copy()
            if i%6==0:
                rows.extend(dict(r,pair_index=i) for r in measure_rectangle(rgb,arrays['depth'],support,roi,reference_mm))
    report=dict(schema='x3.c1.subpixel_trial.v1',declared_mode=declared_mode,
        metadata_subpixel_counts=dict(modes),roi=roi,interior_roi=patch_roi,
        original_audit=json.loads((directory/'audit.json').read_text()),
        analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        calibrations=calibrations,range=range_evidence(patches,reference_mm),
        per_edge={side:separate_edge_evidence([r for r in rows if r['side']==side])
                  for side in ['left','right','top','bottom']},samples=rows)
    if rgb_first is not None:
        import cv2
        x0,y0,x1,y1=roi
        a=cv2.cvtColor(rgb_first[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY).astype('float32')
        b=cv2.cvtColor(rgb_last[y0:y1,x0:x1],cv2.COLOR_BGR2GRAY).astype('float32')
        shift,response=cv2.phaseCorrelate(a,b)
        report['rgb_first_last_diagnostic']=dict(translation_estimate_px=list(shift),response=response,
            mean_absolute_intensity_difference=float(np.abs(a-b).mean()),
            note='Diagnostic for within-run scene changes; does not prove identical cross-run pose/exposure.')
    write_json(directory/'subpixel-report.json',report)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('dataset',type=Path)
    p.add_argument('--roi',type=int,nargs=4,required=True)
    p.add_argument('--interior-roi',type=int,nargs=4,required=True)
    p.add_argument('--distance-mm',type=float,required=True)
    p.add_argument('--declared-mode',choices=['off','on'],required=True)
    a=p.parse_args()
    r=analyze(a.dataset,a.roi,a.interior_roi,a.distance_mm,a.declared_mode)
    print(json.dumps({k:v for k,v in r['range'].items() if k!='frames'},indent=2))


if __name__=='__main__':main()
