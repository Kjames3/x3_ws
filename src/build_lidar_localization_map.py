#!/usr/bin/env python3
"""Build a connected, held-out diagnostic map from the five-station capture.

Experimental graph tooling is intentionally confined to this offline script.
Odd-indexed clouds within each station are never used in graph fitting/map.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'artifacts/registration-2026-09-05'),str(ROOT/'src/yahboomcar_bringup')]
import posegraph as pg
from real_study import station_poses,clip
from study import build_map
from yahboomcar_bringup.lidar_registration import component,transform

def main():
 p=argparse.ArgumentParser();p.add_argument('--capture',default=str(ROOT/'artifacts/registration-2026-09-05/drive_capture.npz'));p.add_argument('--out',default=str(ROOT/'artifacts/localization-2026-09-06'));a=p.parse_args()
 out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
 z=np.load(a.capture);pts=z['points'];cid=z['cloud_id'];meta=z['cloud_meta'];poses=station_poses(z['odom'])
 training=[];held=[]
 for i in range(len(poses)):
  ids=meta[meta[:,1]==i,0].astype(int);training.extend(ids[::2]);held.extend(ids[1::2])
 cs={int(m[0]):int(m[1]) for m in meta};stations=np.array([cs[int(c)] for c in cid])
 train=np.isin(cid,training)
 clouds={i:clip(pts[(stations==i)&train]) for i in range(len(poses))}
 # Preserve the actual low-overlap pair as a deployment regression fixture.
 ref,normal=build_map(clouds[4])
 np.savez_compressed(out/'low_overlap.npz',points=ref,normals=normal,query=clouds[3],prior=np.linalg.inv(pg.se2_to_T(poses[4]))@pg.se2_to_T(poses[3]))
 edges,rejected=pg.build_edges(clouds,poses,np.random.default_rng(0))
 # Add yaw reverse-consistency check; old graph only checked translation.
 maps={i:build_map(c) for i,c in clouds.items()}
 kept=[]
 for e in edges:
  i,j=e['i'],e['j'];f,_,_=pg.register(clouds[j][::5],*maps[i],np.linalg.inv(pg.se2_to_T(poses[i]))@pg.se2_to_T(poses[j]))
  r,_,_=pg.register(clouds[i][::5],*maps[j],np.linalg.inv(f))
  angle=abs(float(pg.se2(f@r)[2]));e['reverse_yaw_rad']=angle
  if angle>np.deg2rad(3):e['rejected_because']='reverse_yaw';rejected.append(e)
  else:kept.append(e)
 edges=kept;connected=component(edges);excluded=sorted(set(range(len(poses)))-set(connected))
 if len(connected)<2:raise RuntimeError('No connected multi-station map')
 optimized=pg.optimize(poses,edges)
 points,normals=build_map(np.concatenate([transform(clouds[i],pg.se2_to_T(optimized[i])) for i in connected]))
 np.savez_compressed(out/'map.npz',points=points,normals=normals,frame=np.array('lidar_map'),connected=np.array(connected),excluded=np.array(excluded),poses=optimized,training_cloud_ids=np.array(training),heldout_cloud_ids=np.array(held))
 manifest={'connected':connected,'excluded':excluded,'map_points':len(points),'edges':edges,'rejected':rejected,'training_cloud_ids':list(map(int,training)),'heldout_cloud_ids':list(map(int,held)),'reference':'training-graph consistency, NOT ground truth','pose_model':'SE2 on 3D geometry','poses':optimized.tolist()}
 (out/'map_manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({k:manifest[k] for k in ['connected','excluded','map_points']},indent=2))
if __name__=='__main__':main()
