#!/usr/bin/env python3
"""Offline paired benchmark; requires ROS and a c1_dataset capture."""
import sys,time,json,importlib.util
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(Path(__file__).resolve().parent),str(ROOT/'src')]
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('dataset',type=Path)
parser.add_argument('--before',type=Path,required=True,help='Baseline refine module saved before optimization')
options=parser.parse_args()
from c1_dataset import load_pair,rgb_on_depth_grid
from oak_color_cloud_probe import prepare_geometry
from oak_color_cloud_refine import refine,balanced_pixels
spec=importlib.util.spec_from_file_location('before',str(options.before));old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
p=options.dataset
samples=[];rectify=[]
for i in range(0,60,6):
 a,r=load_pair(p,i);t=time.perf_counter();rgb,s=rgb_on_depth_grid(a,r);rectify.append((time.perf_counter()-t)*1000);c=r['metadata']['calibration']
 samples.append((rgb,a['depth'],s,np.asarray(c['depth_k']).reshape(3,3),c['depth_inv_offset_per_m']))
for args in samples:
 before=old.refine(*args);after=refine(*args)
 for a,b in zip(before,after):np.testing.assert_array_equal(a,b)
times={'before':[],'after':[],'geometry_mask':[],'sampling':[]}
for repeat in range(6):
 for args in samples:
  for name,fn in ([('before',old.refine),('after',refine)] if repeat%2==0 else [('after',refine),('before',old.refine)]):
   t=time.perf_counter();fn(*args);ms=(time.perf_counter()-t)*1000
   if repeat:times[name].append(ms)
  rgb,d,s,k,c=args;t=time.perf_counter();z,v,m=prepare_geometry(rgb,d,s,c);ms=(time.perf_counter()-t)*1000
  if repeat:times['geometry_mask'].append(ms)
  t=time.perf_counter();balanced_pixels(v,6000);ms=(time.perf_counter()-t)*1000
  if repeat:times['sampling'].append(ms)
report={'samples_per_variant':50,'exact_output_equivalence_frames':10,'timings_ms':times,'rectification_uncached_ms':rectify,'summary':{k:{'median':float(np.median(v)),'p95':float(np.percentile(v,95))} for k,v in times.items()},'excludes':'Disk read, rectification, serialization, network, browser; inputs preloaded; alternating order'}
(p/'optimized-profile.json').write_text(json.dumps(report,indent=2));print(json.dumps(report['summary'],indent=2));print('uncached rectify median',np.median(rectify))
