#!/usr/bin/env python3
"""Plot offline cloud probe retention; no live robot inputs."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from oak_color_cloud_probe import project
from oak_color_cloud_refine import refine

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('sample',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();s=np.load(a.sample)
fig,axes=plt.subplots(1,3,figsize=(12,5))
axes[0].imshow(s['rgb'][...,::-1]);axes[0].set_title('Rectified RGB reference')
for ax,method in zip(axes[1:],["old", "refined"]):
 if method == 'old':
  xyz,colors,mask,_=project(s['rgb'],s['depth'],s['support'],s['k'],float(s['correction']),edge_margin=12)
 else:
  xyz,colors,mask,_=refine(s['rgb'],s['depth'],s['support'],s['k'],float(s['correction']))
 # Reproject the actual budgeted cloud into its acquisition view. This tests
 # recognizable detail after the 6000-point cap, not just the dense mask.
 u=xyz[:,0]/xyz[:,2]*s['k'][0,0]+s['k'][0,2]
 v=xyz[:,1]/xyz[:,2]*s['k'][1,1]+s['k'][1,2]
 ax.set_facecolor('#20242c');ax.scatter(u,v,c=colors/255,s=4,linewidths=0)
 ax.set_title(f'{method}: {len(xyz)} points')
for ax in axes:
 ax.set_xlim(105,280);ax.set_ylim(350,120);ax.set_aspect('equal');ax.set_xlabel('Image x (px)')
axes[0].set_ylabel('Image y (px)')
fig.suptitle('Recorded raised box — acquisition-view projection, no interpolation')
fig.tight_layout();fig.savefig(a.output,dpi=140)
