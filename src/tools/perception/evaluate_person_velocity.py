#!/usr/bin/env python3
"""Offline person-input MLP comparison and measurement-uncertainty calibration.

`tune` reads only repeat-1 caches. `evaluate` freezes that choice, then reads
repeat-2/3 recordings. References share the camera: agreement is not ground truth.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
_REPO_ROOT = _Path(__file__).resolve().parents[3]
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
from collections import defaultdict
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
import torch

import c3_person_tracker as pt
import c3_score as cs
import c3_replay as rp
from c3_live import ARM_C_CONFIG
from person_velocity_mlp import PersonVelocityMLP

FRONTENDS = ('pose', 'seg-merged')
ROOT = _REPO_ROOT


def read(path):
    return json.loads(path.read_text())


def write(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, allow_nan=False,
        default=lambda value: value.item() if isinstance(value, np.generic) else value.tolist()) + '\n')


def sampled(frames):
    """Newest recorded acquisition at each 10 Hz tick, once per frame."""
    j = 0
    last = None
    for tick in range(frames[0]['t_ns'], frames[-1]['t_ns'] + 1, 100_000_000):
        while j + 1 < len(frames) and frames[j + 1]['t_ns'] <= tick:
            j += 1
        if j != last:
            yield frames[j]
            last = j


def measurement_cov(m, cfg):
    return pt.rotate_cov_to_odom(pt.measurement_cov_camera(
        m['z'], cfg['meas_sigma_floor_m'], m['spread'],
        cfg['depth_noise_coef'], cfg['measurement_spread_scale']), np.array(m['rot']))


def replay(frames, config, mlp=None):
    tracker = pt.KalmanTracker(config)
    kalman_out, mlp_out = {}, {}
    origin = frames[0]['t_ns']
    if mlp is not None:
        mlp.reset()
    for f in sampled(frames):
        stamp = f['t_ns']
        kalman_out[stamp], mlp_out[stamp] = [], []
        if not f['has_pose']:
            tracker.reset()
            if mlp is not None:
                mlp.reset()
            continue
        meas = [(np.array(m['xy']), measurement_cov(m, tracker.config)) for m in f['meas']]
        outs = tracker.update((stamp - origin) / 1e9, meas)
        if mlp is not None:
            mlp.retain([o['id'] for o in outs])
        for o in outs:
            if not o['measured']:
                continue
            m = f['meas'][o['measurement_index']]
            if o['confirmed']:
                kalman_out[stamp].append(dict(o, t_ns=stamp, z=m['z']))
            if mlp is not None:
                # Feed observations even before confirmation, but never predictions.
                pred = mlp.update(o['id'], (stamp-origin)/1e9, m['xy'], m['rot'])
                if pred is not None and o['confirmed']:
                    mlp_out[stamp].append(dict(pred, t_ns=stamp, z=m['z']))
    return kalman_out, mlp_out


def reference(run, inside=True):
    name = 'c3-reference-inside.json' if inside else 'c3-reference.json'
    path = run / name
    if not path.exists():
        path = run / 'c3-reference.json'
    old = cs.REFERENCE_FILE
    cs.REFERENCE_FILE = path.name
    try:
        return cs.reference_track(run)
    finally:
        cs.REFERENCE_FILE = old


def rows_for(by_t, ref):
    ts, positions, speeds = ref
    rows = []
    for t, outs in by_t.items():
        k = int(np.argmin(np.abs(ts-t)))
        person = positions[k] if abs(int(ts[k])-t) <= cs.TIME_MATCH_NS else None
        match = None
        others = list(outs)
        if person is not None and outs:
            ds = [math.hypot(o['x']-person['x'], o['y']-person['y']) for o in outs]
            j = int(np.argmin(ds))
            if ds[j] <= cs.MATCH_M:
                match = outs[j]
                others = outs[:j]+outs[j+1:]
        rows.append(dict(t=t, person=person, ref_speed=speeds[k] if person else np.nan,
                         matched=match, others=others))
    return rows


def uncertainty(by_t, ref, standing, accel):
    """Conditional matched-track calibration; coverage/recall scored separately."""
    rows = rows_for(by_t, ref)
    if standing and rows:
        start = rows[0]['t']
        rows = [r for r in rows if r['t'] - start >= 2_000_000_000]
    pairs = [(r, r['matched']) for r in rows if r['matched'] is not None]
    values = defaultdict(list)
    if standing and pairs:
        # Static spread about each track mean measures jitter, not absolute bias.
        means = {}
        for tid in {o['id'] for _, o in pairs}:
            points = [[o['x'],o['y']] for _,o in pairs if o['id']==tid]
            if len(points) >= 20:
                means[tid] = np.mean(points, axis=0)
    else:
        means = {}
    ts, pos, _ = ref
    for r, o in pairs:
        xy = np.array([o['x'],o['y']]); v = np.array([o['vx'],o['vy']])
        cov = np.asarray(o['cov'])
        for horizon in (0., .5, 1.):
            if standing:
                if o['id'] not in means:
                    continue
                target = means[o['id']]
            else:
                k = int(np.argmin(np.abs(ts-(r['t']+horizon*1e9))))
                if abs(ts[k]-(r['t']+horizon*1e9)) > cs.TIME_MATCH_NS or pos[k] is None:
                    continue
                target = np.array([pos[k]['x'],pos[k]['y']])
            f = np.hstack([np.eye(2), horizon*np.eye(2)])
            pc = f @ cov @ f.T + np.eye(2)*accel**2*horizon**4/4
            err = xy + horizon*v - target
            nees = float(err @ np.linalg.solve(pc,err))
            logdet = float(np.linalg.slogdet(pc)[1])
            values[horizon].append([float(err@err), nees, logdet+nees, float(np.trace(pc))])
    out = {}
    for h, vals in values.items():
        a = np.asarray(vals)
        out[str(h)] = dict(n=len(vals), rmse_m=float(np.sqrt(a[:,0].mean())),
                          cover95=float(np.mean(a[:,1] <= 5.991)), nees=float(a[:,1].mean()),
                          nll=float(.5*a[:,2].mean()), sigma_m=float(np.sqrt(a[:,3].mean())))
    return out


def metrics(by_t, ref, scenario):
    rows = rows_for(by_t, ref)
    if scenario == 'stand' and rows:
        start = rows[0]['t']
        rows = [r for r in rows if r['t'] - start >= 2_000_000_000]
    eligible = [r for r in rows if r['person'] is not None]
    matched = [r for r in eligible if r['matched'] is not None]
    moving = [r for r in matched if np.isfinite(r['ref_speed']) and r['ref_speed'] > .3]
    speederr = [abs(r['matched']['speed']-r['ref_speed']) for r in moving]
    false = [r['matched']['speed'] > .15 for r in matched] if scenario == 'stand' else []
    other = [o for r in rows for o in r['others']]
    return dict(reference_frames=len(eligible), matched=len(matched),
                recall=len(matched)/len(eligible) if eligible else None,
                moving_n=len(moving), speed_mae=float(np.mean(speederr)) if speederr else None,
                static_false_fraction=float(np.mean(false)) if false else None,
                nonperson_outputs=len(other), nonperson_above015=sum(o['speed']>.15 for o in other),
                events=cs.stop_go_delays(rows) if scenario == 'startstop' else {})


def tune(data, out, split):
    configs = {'baseline': dict(ARM_C_CONFIG)}
    for coef, floor, spread in itertools.product((.0025,.009,.011),(.02,.05),(.25,.5,1.)):
        name=f'coef{coef:g}_floor{floor:g}_spread{spread:g}'
        configs[name]=dict(ARM_C_CONFIG,depth_noise_coef=coef,meas_sigma_floor_m=floor,
                           measurement_spread_scale=spread)
    # This stage intentionally never opens held-out measurement/reference files.
    results = {fe:{name:[] for name in configs} for fe in FRONTENDS}
    for name in split['tuning']:
        run=data/name; ref=reference(run); scenario=cs.scenario_of(run)
        for fe in FRONTENDS:
            frames=read(run/f'c3-meas-{fe}.json')
            for cname,cfg in configs.items():
                tracks,_=replay(frames,cfg)
                results[fe][cname].append(dict(run=name,
                    metrics=metrics(tracks,ref,scenario),
                    uncertainty=uncertainty(tracks,ref,scenario=='stand',cfg['accel_sigma_mps2'])))
        print('tuned',name,flush=True)
    chosen={}; rankings={}
    for fe in FRONTENDS:
        ranking=[]
        base=results[fe]['baseline']
        for name, runs in results[fe].items():
            scores=[]; eligible=True
            for r,b in zip(runs,base):
                m,bm=r['metrics'],b['metrics']
                if m['recall'] is not None and m['recall'] < bm['recall']-.02:
                    eligible=False
                if m['static_false_fraction'] is not None and m['static_false_fraction'] > bm['static_false_fraction']+.02:
                    eligible=False
                if m['speed_mae'] is not None and m['speed_mae'] > bm['speed_mae']+.03:
                    eligible=False
                u=r['uncertainty']
                if '0.0' in u and '0.5' in u:
                    # Penalize a too-small covariance via proper Gaussian score.
                    scores.append(u['0.0']['nll']+.5*u['0.5']['nll'])
                    if u['0.5']['cover95'] < .90:
                        eligible=False
            ranking.append(dict(name=name, eligible=eligible,
                                score=float(np.mean(scores)) if scores else None))
        valid=[r for r in ranking if r['eligible'] and r['score'] is not None]
        winner=min(valid,key=lambda r:r['score'])['name'] if valid else 'baseline'
        chosen[fe]=dict(name=winner,config=configs[winner])
        rankings[fe]=sorted(ranking,key=lambda r:r['score'] if r['score'] is not None else float('inf'))
    write(out/'tuning-results.json',results)
    write(out/'selected.json',dict(split=split,selection='repeat-1 only; NLL with per-run recall/static/speed/coverage constraints',
                                   candidates=chosen,rankings=rankings))
    print(json.dumps(chosen,indent=2))


def blob_frames(run, cache):
    provenance = {}
    for path in (run/'pair-index.json', run/'depth-export.json',
                 run/'snapshot/config/camera_ground_plane.json', ROOT/'src/velocity_estimator.py'):
        if path.exists():
            provenance[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    if cache.exists():
        cached = read(cache)
        if isinstance(cached, dict) and cached.get('provenance') == provenance:
            return cached['frames']
    from c1_dataset import load_pair
    model=rp.make_v3(run)
    rows=read(run/'pair-index.json'); frames=[]
    packed = np.load(run/'depth-frames.npz') if (run/'depth-frames.npz').exists() else None
    for i,row in enumerate(rows):
        if packed is not None:
            arrays, checked = {'depth': packed[f'frame_{i}']}, row
        else:
            arrays, checked=load_pair(run,i)
        cents=model._extract_depth_centroids(None,rp.depth_metres(arrays,checked),rp.intrinsics(row))
        transform=row.get('odom_from_camera'); meas=[]
        if transform:
            for point in cents:
                xy,rot=pt.camera_point_to_odom(point,transform)
                meas.append(dict(xy=xy.tolist(),rot=rot.tolist(),z=point[2],spread=0.))
        frames.append(dict(t_ns=row['depth']['stamp_ns'],has_pose=transform is not None,meas=meas))
    if packed is not None:
        packed.close()
    write(cache,dict(provenance=provenance,frames=frames))
    return frames


def evaluate(data,out,split,with_blobs):
    selected=read(out/'selected.json')  # frozen before any held-out scoring
    mlp=PersonVelocityMLP(model_path=str(ROOT/'src/velocity_mlp_v3.torchscript'))
    results={}; detailed={}
    for name in split['held_out']+split['empty_check']:
        run=data/name; ref=reference(run); full_ref=reference(run,inside=False); scenario=cs.scenario_of(run)
        arms={}
        for fe in FRONTENDS:
            frames=read(run/f'c3-meas-{fe}.json')
            kal,net=replay(frames,ARM_C_CONFIG,mlp)
            arms[fe+'/kalman']=kal; arms[fe+'/mlp']=net
            candidate=selected['candidates'][fe]['config']
            arms[fe+'/calibrated'],_=replay(frames,candidate)
        if with_blobs:
            frames=blob_frames(run,out/'blob-cache'/f'{name}.json')
            # Same association algorithm as person arms: isolate measurements.
            _,net=replay(frames,ARM_C_CONFIG,mlp)
            arms['blob/mlp']=net
        results[name]={}
        for arm,outputs in arms.items():
            cfg=selected['candidates'][arm.split('/')[0]]['config'] if arm.endswith('/calibrated') else ARM_C_CONFIG
            res=dict(metrics=metrics(outputs,ref,scenario),
                     full_reference_metrics=metrics(outputs,full_ref,scenario))
            if not arm.endswith('/mlp'):
                res['uncertainty']=uncertainty(outputs,ref,scenario=='stand',cfg['accel_sigma_mps2'])
            results[name][arm]=res
        # Speed comparison on timestamps where ALL arms match the same reference.
        matched={arm:{r['t']:r for r in rows_for(outputs,ref) if r['matched'] is not None}
                 for arm,outputs in arms.items()}
        common=set.intersection(*(set(v) for v in matched.values()))
        for arm,m in matched.items():
            moving=[m[t] for t in common if np.isfinite(m[t]['ref_speed']) and m[t]['ref_speed']>.3]
            results[name][arm]['common_speed_mae']=float(np.mean([
                abs(r['matched']['speed']-r['ref_speed']) for r in moving])) if moving else None
            results[name][arm]['common_moving_n']=len(moving)
        for fe in FRONTENDS:
            pair = [fe+'/kalman', fe+'/mlp']
            common_pair = set(matched[pair[0]]) & set(matched[pair[1]])
            for arm in pair:
                moving = [matched[arm][t] for t in common_pair
                          if np.isfinite(matched[arm][t]['ref_speed']) and matched[arm][t]['ref_speed'] > .3]
                results[name][arm]['paired_moving_n'] = len(moving)
                results[name][arm]['paired_speed_mae'] = float(np.mean([
                    abs(r['matched']['speed']-r['ref_speed']) for r in moving])) if moving else None
        detailed[name]={arm:[dict(t_ns=t,outputs=values) for t,values in outputs.items()]
                        for arm,outputs in arms.items()}
        write(out/'held-out-results.json',results)
        write(out/'outputs'/f'{name}.json',detailed[name])
        print('evaluated',name,flush=True)
    write(out/'evaluation-provenance.json',dict(
        selected_sha256=hashlib.sha256((out/'selected.json').read_bytes()).hexdigest(),
        model_sha256=hashlib.sha256((ROOT/'src/velocity_mlp_v3.torchscript').read_bytes()).hexdigest(),
        reference='c3-reference-inside.json where available; shared-sensor proxy',
        blob_association='same Kalman association as person arms; not deployed blob tracker',
        split=split))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('stage',choices=['tune','evaluate'])
    ap.add_argument('--data',type=Path,default=ROOT/'artifacts/c3-captures-segpose')
    ap.add_argument('--output',type=Path,default=ROOT/'artifacts/person-velocity-2026-10-12')
    ap.add_argument('--with-blobs',action='store_true')
    args=ap.parse_args();torch.set_num_threads(1)
    split=read(args.output/'split.json')
    if args.stage=='tune': tune(args.data,args.output,split)
    else: evaluate(args.data,args.output,split,args.with_blobs)

if __name__=='__main__':main()
