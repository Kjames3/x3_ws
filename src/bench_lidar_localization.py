#!/usr/bin/env python3
"""Held-out, real-cloud diagnostic benchmark; graph pose is not ground truth."""
import argparse,json,sys
from pathlib import Path
from collections import Counter
from dataclasses import replace
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Matcher,Limits,pose_matrix

def main():
 p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--capture',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 z=np.load(a.capture);m=np.load(a.map);matcher=Matcher(m['points'],m['normals'])
 points=z['points'];cid=z['cloud_id'];meta=z['cloud_meta'];held=set(m['heldout_cloud_ids'].tolist());connected=set(m['connected'].tolist());poses=m['poses'];rows=[]
 for entry in meta:
  c,s=int(entry[0]),int(entry[1])
  if c not in held or s not in connected:continue
  ref=pose_matrix(*poses[s]);initial=pose_matrix(poses[s,0]+.08,poses[s,1]-.06,poses[s,2]+np.deg2rad(2))
  T,metrics=matcher.match(points[cid==c],initial)
  metrics.update(cloud=c,station=s,graph_disagreement_m=float(np.linalg.norm(T[:2,3]-ref[:2,3])),graph_disagreement_yaw_deg=float(np.degrees(abs(np.arctan2(np.sin(np.arctan2(T[1,0],T[0,0])-poses[s,2]),np.cos(np.arctan2(T[1,0],T[0,0])-poses[s,2]))))))
  rows.append(metrics)
 # Out-of-map and non-observable fixtures, never considered localization successes.
 q=points[cid==int(meta[0,0])]
 negatives=[]
 for name,cloud,prior in [('empty',np.empty((0,3)),pose_matrix()),('unmapped_prior',q,pose_matrix(30,30)),('nonfinite',np.full((200,3),np.nan),pose_matrix())]:
  _,r=matcher.match(cloud,prior);negatives.append(dict(fixture=name,**r))
 fixture=Path(a.map).with_name('low_overlap.npz')
 if fixture.exists():
  f=np.load(fixture);_,r=Matcher(f['points'],f['normals']).match(f['query'],f['prior']);negatives.append(dict(fixture='recorded_station_3_to_4',**r))
 good=[r for r in rows if r['accepted']]
 def pct(vals):return np.percentile(vals,[50,95,100]).tolist() if vals else []
 summary={'queries':len(rows),'accepted':len(good),'rejections':dict(Counter(reason for r in rows for reason in r['reasons'])),'runtime_ms_p50_p95_max':pct([r['runtime_s']*1000 for r in rows]),'accepted_graph_disagreement_m_p50_p95_max':pct([r['graph_disagreement_m'] for r in good]),'accepted_graph_disagreement_yaw_deg_p50_p95_max':pct([r['graph_disagreement_yaw_deg'] for r in good]),'negative_fixtures':negatives,'note':'Held-out clouds from the same stationary capture; reference is training pose graph, not independent truth. Explicit map-aligned 10 cm / 2 degree prior.'}
 Path(a.out).write_text(json.dumps({'summary':summary,'rows':rows},indent=2));print(json.dumps(summary,indent=2))
 if any(r['accepted'] for r in negatives):raise RuntimeError('negative fixture accepted')
if __name__=='__main__':main()
