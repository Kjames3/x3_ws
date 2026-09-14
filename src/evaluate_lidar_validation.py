#!/usr/bin/env python3
"""Frozen-map validation on a new stop-and-go capture.

Offline bootstrap uses ONLY first half of station 0 to find map alignment.
This is not the production initializer, and its pose is not ground truth.
Subsequent clouds track sequentially using exact stamped odometry and the
unchanged production gates. No new geometry is added to the frozen map.
"""
import argparse,json,sys,time,hashlib
from pathlib import Path
from dataclasses import replace
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Matcher,Limits,pose_matrix,transform


def main():
 p=argparse.ArgumentParser();p.add_argument('--capture',required=True);p.add_argument('--map',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 z=np.load(a.capture);m=np.load(a.map);pts=z['points'];cid=z['cloud_id'];meta=z['cloud_meta'];co=z['cloud_odom']
 assert len(meta)==len(co) and np.array_equal(meta[:,0],co[:,0]) and np.array_equal(meta[:,2],co[:,1])
 assert np.isfinite(pts).all() and np.all(np.diff(meta[:,2])>0)
 # Index once; recorder groups points by cloud.
 _,starts,counts=np.unique(cid,return_index=True,return_counts=True)
 clouds={int(c):pts[start:start+count] for c,start,count in zip(np.unique(cid),starts,counts)}
 odoms={int(row[0]):pose_matrix(*row[2:5]) for row in co}
 first=meta[meta[:,1]==0];bootstrap_ids=first[:len(first)//2,0].astype(int);anchor=odoms[int(first[0,0])]
 query=np.concatenate([transform(clouds[c],np.linalg.inv(anchor)@odoms[c]) for c in bootstrap_ids])
 search=Matcher(m['points'],m['normals'],replace(Limits(),max_iterations=45,correspondence_m=.8,max_correction_m=3.,max_correction_yaw_rad=np.pi,max_runtime_s=2.))
 candidates=[];began=time.monotonic()
 # Search map-station neighborhoods; only an offline proposal generator.
 seeds=[]
 for station in m['connected']:
  base=m['poses'][station]
  for dx,dy in [(0,0),(.65,0),(-.65,0),(0,.65),(0,-.65)]:
   for yaw in np.arange(-np.pi,np.pi,np.pi/6):seeds.append(pose_matrix(base[0]+dx,base[1]+dy,yaw))
 for initial in seeds:
  T,r=search.match(query,initial)
  if r['accepted']:
   candidates.append({'T':T.tolist(),'metrics':r})
 print('bootstrap search %d seeds, %d admitted, %.1f s'%(len(seeds),len(candidates),time.monotonic()-began),flush=True)
 if not candidates:raise RuntimeError('No reliable bootstrap candidate; need a known map pose')
 candidates.sort(key=lambda c:(-c['metrics']['overlap'],c['metrics']['nn_rmse_m']))
 best=candidates[0];T0=np.array(best['T']);clusters=[]
 for c in candidates:
  T=np.array(c['T']);yaw=np.arctan2(T[1,0],T[0,0])
  if not any(np.linalg.norm(T[:2,3]-np.array(k['T'])[:2,3])<.20 and abs(np.arctan2(np.sin(yaw-np.arctan2(k['T'][1][0],k['T'][0][0])),np.cos(yaw-np.arctan2(k['T'][1][0],k['T'][0][0]))))<np.deg2rad(10) for k in clusters):clusters.append(c)
 print('bootstrap distinct clusters',len(clusters),'best',best,flush=True)
 matcher=Matcher(m['points'],m['normals']);correction=T0@np.linalg.inv(anchor);rows=[]
 for row in meta:
  c,s=int(row[0]),int(row[1])
  if c in bootstrap_ids:continue
  O=odoms[c];prior=correction@O;T,r=matcher.match(clouds[c],prior)
  if r['accepted']:correction=T@np.linalg.inv(O)
  rows.append(dict(cloud=c,station=s+1,stamp=float(row[2]),pose=T.tolist() if r['accepted'] else None,prior=prior.tolist(),**r))
 stations=[]
 for s in range(8):
  sub=meta[meta[:,1]==s];o=co[np.isin(co[:,0],sub[:,0])];results=[r for r in rows if r['station']==s+1];good=[r for r in results if r['accepted']]
  entry={'station':s+1,'recorded_clouds':len(sub),'tested':len(results),'accepted':len(good),'rejections':dict(Counter(x for r in results for x in r['reasons'])),'max_acquisition_gap_s':float(np.diff(sub[:,2]).max()),'odom_translation_span_m':float(np.linalg.norm(o[:,2:4]-o[0,2:4],axis=1).max()),'odom_yaw_span_deg':float(np.degrees(np.ptp(np.unwrap(o[:,4]))))}
  if good:
   xy=np.array([np.array(r['pose'])[:2,3] for r in good]);entry['accepted_xy_median']=np.median(xy,axis=0).tolist();entry['accepted_xy_radius_from_median_p95_m']=float(np.percentile(np.linalg.norm(xy-np.median(xy,axis=0),axis=1),95))
  stations.append(entry)
 summary={'points':len(pts),'clouds':len(meta),'bootstrap_clouds':len(bootstrap_ids),'tested':len(rows),'accepted':sum(r['accepted'] for r in rows),'bootstrap_clusters':clusters,'stations':stations,'runtime_ms_p50_p95_max':np.percentile([r['runtime_s']*1000 for r in rows],[50,95,100]).tolist(),'map_sha256':hashlib.sha256(Path(a.map).read_bytes()).hexdigest(),'capture_sha256':hashlib.sha256(Path(a.capture).read_bytes()).hexdigest(),'note':'Offline bootstrap from first half of first station. Frozen map; sequential odometry prior; no ground truth. Final pose intentionally differs from start; no loop-closure score.'}
 Path(a.out).write_text(json.dumps({'summary':summary,'rows':rows},indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='bootstrap_clusters'},indent=2),flush=True)
if __name__=='__main__':main()
