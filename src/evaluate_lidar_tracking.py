#!/usr/bin/env python3
"""Evaluate temporal tracking on the eight-station development recording.
No map changes or station-specific pose initialization. Explicit pause/resume
between recorded stations reflects this capture's intentional no-cloud driving.
"""
import argparse,json,sys,time,hashlib
from pathlib import Path
from dataclasses import asdict
from collections import Counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_registration import Matcher,pose_matrix
from yahboomcar_bringup.lidar_tracking import Tracker


def main():
 p=argparse.ArgumentParser();p.add_argument('--capture',required=True);p.add_argument('--map',required=True);p.add_argument('--bootstrap',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 z=np.load(a.capture);m=np.load(a.map);old=json.loads(Path(a.bootstrap).read_text());T0=np.array(old['summary']['bootstrap_clusters'][0]['T']);meta=z['cloud_meta'];co=z['cloud_odom'];pts=z['points'];cid=z['cloud_id']
 assert np.array_equal(meta[:,0],co[:,0]) and np.array_equal(meta[:,2],co[:,1])
 bootstrap=int(old['summary']['bootstrap_clouds']);t=Tracker(Matcher(m['points'],m['normals']));t.initialize(T0@np.linalg.inv(pose_matrix(*co[0,2:5])),meta[bootstrap-1,2]);rows=[];station=0
 for i in range(bootstrap,len(meta)):
  c,s=int(meta[i,0]),int(meta[i,1]);stamp=float(meta[i,2]);O=pose_matrix(*co[i,2:5]);q=pts[cid==c]
  if s!=station:t.pause(True);t.pause(False);station=s
  pitch=float(np.interp(stamp+.069,z['joints'][:,0],z['joints'][:,1]))
  before=time.perf_counter();pose,r=t.process(q,O,stamp,pitch);runtime=time.perf_counter()-before
  rows.append(dict(cloud=c,station=s+1,stamp=stamp,pose=None if pose is None else pose.tolist(),runtime_s=runtime,**r))
 stats=[]
 for s in range(1,9):
  rs=[r for r in rows if r['station']==s];good=[r for r in rs if r['accepted']];r=dict(station=s,tested=len(rs),published=len(good),states=dict(Counter(x['state'] for x in rs)),reasons=dict(Counter(x['reason'] for x in rs)))
  if good:
   xy=np.array([np.array(x['pose'])[:2,3] for x in good]);r['xy_radius_p95_m']=float(np.percentile(np.linalg.norm(xy-np.median(xy,axis=0),axis=1),95));r['xy_median']=np.median(xy,axis=0).tolist();r['first_output_delay_s']=good[0]['stamp']-rs[0]['stamp']
  stats.append(r)
 summary=dict(policy=asdict(t.policy),tested=len(rows),published=sum(r['accepted'] for r in rows),reacquisitions=sum(r['reason']=='reacquired' for r in rows),runtime_ms_p50_p95_max=np.percentile([r['runtime_s']*1000 for r in rows],[50,95,100]).tolist(),stations=stats,map_sha256=hashlib.sha256(Path(a.map).read_bytes()).hexdigest(),capture_sha256=hashlib.sha256(Path(a.capture).read_bytes()).hexdigest(),note='Development-data evaluation, not independent validation or absolute accuracy. First 35 clouds bootstrap as in baseline. Known intentional pauses between stations; no per-station map pose resets.')
 Path(a.out).write_text(json.dumps(dict(summary=summary,rows=rows),indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
