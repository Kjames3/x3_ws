#!/usr/bin/env python3
"""Replay lidar_persons.ndjson through the tracker, with and without seeded candidates.

    python3 src/tools/perception/seeded_tracker_replay.py RUN_DIR [RUN_DIR ...]
"""
import sys,json,math; import pathlib; sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[2]))
from lidar_person_tracker import LidarPersonTracker, merge_seeded
SF=['x','y','x_lo','x_hi','y_lo','y_hi','z_lo','z_hi','n']
for run in sys.argv[1:]:
    L=[json.loads(l) for l in open(run+'/lidar_persons.ndjson')]
    for use in (False,True):
        T=LidarPersonTracker(); cam=conf=near=0; prev=None; ids=set()
        for f in L:
            now=f['stamp']; c=[dict(zip(f['fields'],r)) for r in f['candidates']]
            if use:
                s=f.get('seeded')
                if s: prev=(s,now)
                if prev: c=merge_seeded(c,prev[0],SF,prev[1],now)
            R=T.update(now,c,f['camera'],f['packet_age'])
            ok=[r for r in R if r['confirmed']]; ids|={r['id'] for r in ok}
            if f['camera']:
                cam+=1; conf+=bool(ok)
                a=f['camera'][0]; near+=any(math.hypot(r['odom_x']-a['odom_x'],r['odom_y']-a['odom_y'])<0.6 for r in ok)
        print(run.split('/')[-1],'seeded' if use else 'base  ','camera frames',cam,'confirmed track %.1f%%'%(100*conf/max(cam,1)),'on the person %.1f%%'%(100*near/max(cam,1)),'ids',len(ids),'births',T.stats['births'],'held %.1fs'%T.stats['lidar_held_s'])
