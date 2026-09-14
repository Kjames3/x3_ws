"""Offline whole-station reacquisition proposals; never counted as tracking."""
import sys,json,time
from pathlib import Path
from dataclasses import replace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Matcher,Limits,pose_matrix,transform
folder=ROOT/'artifacts/localization-validation';z=np.load(folder/'localization_validation_01.npz');m=np.load(ROOT/'artifacts/localization-2026-09-06/map.npz');meta=z['cloud_meta'];co=z['cloud_odom'];pts=z['points'];cid=z['cloud_id'];tracking=json.loads((folder/'frozen_map_result.json').read_text());T0=np.array(tracking['summary']['bootstrap_clusters'][0]['T']);C0=T0@np.linalg.inv(pose_matrix(*co[0,2:5]));out=[]
matcher=Matcher(m['points'],m['normals'],replace(Limits(),max_iterations=45,correspondence_m=.8,max_correction_m=3.,max_correction_yaw_rad=np.pi,max_runtime_s=2.))
for station in [4,6,7]:
 rows=meta[meta[:,1]==station];ids=rows[:,0].astype(int);O=co[np.isin(co[:,0],ids)];anchor=pose_matrix(*O[0,2:5]);q=np.concatenate([transform(pts[cid==int(o[0])],np.linalg.inv(anchor)@pose_matrix(*o[2:5])) for o in O]);candidates=[];began=time.monotonic()
 for old in m['connected']:
  base=m['poses'][old]
  for dx,dy in [(0,0),(.65,0),(-.65,0),(0,.65),(0,-.65)]:
   for yaw in np.arange(-np.pi,np.pi,np.pi/6):
    T,r=matcher.match(q,pose_matrix(base[0]+dx,base[1]+dy,yaw))
    if r['accepted']:candidates.append({'pose':T.tolist(),**r})
 candidates.sort(key=lambda c:(-c['overlap'],c['nn_rmse_m']))
 best=candidates[0] if candidates else None
 row={'station':station+1,'search_admitted':len(candidates),'best':best,'search_seconds':time.monotonic()-began,'warning':'Whole-station offline search; consumes all clouds here, not independent truth or online tracking'}
 if best:
  T=np.array(best['pose']);prior=C0@anchor;row['disagreement_from_initial_odom_alignment_m']=float(np.linalg.norm(T[:2,3]-prior[:2,3]));row['alternative_clusters']=[]
  for c in candidates:
   P=np.array(c['pose'])
   if np.linalg.norm(P[:2,3]-T[:2,3])>.3:row['alternative_clusters'].append(c);break
 out.append(row);(folder/'reacquisition_diagnostic.json').write_text(json.dumps(out,indent=2));print(json.dumps(row),flush=True)
