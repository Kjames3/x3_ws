#!/usr/bin/env python3
"""Score seeded clusters with height caps (2026-11-22 couch check).

    python3 src/tools/perception/seeded_zcap_check.py RUN_DIR
"""
import sys,pathlib,json,numpy as np,joblib
sys.path.insert(0,'src/tools/perception'); sys.path.insert(0,'src/yahboomcar_bringup')
import lidar_person_dataset as lpd
from yahboomcar_bringup import cluster_features as cf, person_candidates as pc
clf=joblib.load('artifacts/lidar-person-classifier-2026-11-01/model_base_1121.joblib')['model']
run=pathlib.Path(sys.argv[1]); ov=json.load(open('artifacts/lidar-person-classifier-2026-11-01/label_overrides.json'))
base=lpd.process_run(run,ov); by={}
for i,t in enumerate(base['t']): by.setdefault(float(t),[]).append(i)
caps=[None,1.6,1.4,1.3,1.2,1.1]; P={c:[] for c in caps}; zh=[]; prof=[]
for f in lpd.iter_frames(run):
    if not len(f['cam']): continue
    wx,wy,z=f['wx'],f['wy'],f['z']; o=(f['ox'],f['oy'])
    bi=by.get(float(f['end']),[]); ex=base['xy'][bi] if bi else None
    rows,idx,_=pc.seeded_candidates(wx,wy,z,f['cam'],origin=o,existing=ex,return_points=True)
    for pi in idx:
        zh.append(z[pi].max()); prof.append(np.histogram(z[pi],bins=np.arange(0,2.21,0.2))[0])
        for c in caps:
            q=pi if c is None else pi[z[pi]<c]
            P[c].append(clf.predict_proba([cf.features(wx[q],wy[q],z[q],origin=o)])[0,1] if len(q)>=10 else 0)
print('rows',len(zh),'z_max pct 10/50/90',np.percentile(zh,[10,50,90]).round(2))
print('mean pts per 0.2m z bin from 0:',np.mean(prof,0).round(1))
for c in caps:
    p=np.array(P[c]); print('cap',c,'>0.5 %.1f%%  >0.3 %.1f%%  median p %.3f'%(100*(p>.5).mean(),100*(p>.3).mean(),np.median(p)))
