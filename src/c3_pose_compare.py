#!/usr/bin/env python3
"""Does body pose warn of a start or a turn earlier than the C3 Kalman velocity?

Three stages, because the bag reader needs ROS Python and MediaPipe lives in
the pose-lite venv (Python 3.12):

  python3 src/c3_pose_compare.py export RUN...        # ROS python: crops -> .cache
  .cache/pose-lite/venv/bin/python src/c3_pose_compare.py pose RUN...
  python3 src/c3_pose_compare.py score EVAL_DIR RUN... [--output OUT.json]

export: for every frame with a reference (YOLO11x) person, save the RGB crop
        around that box (padded) on the depth grid, plus the crop offset.
pose:   MediaPipe Pose Landmarker Lite (VIDEO mode) on the crops, landmarks
        mapped back to full-image pixels -> RUN/c3-pose.json.
score:  1. visibility: hips/shoulders/ankles visible (>=0.5) per range bin;
        2. start onset: gait cue (ankle separation / torso length leaving its
           standing baseline) vs arm C speed crossing 0.15 m/s, both relative
           to the reference 'go' time; positive lead = pose earlier;
        3. facing: sign of nose - hip-midpoint (image x) vs the direction the
           person moves next, before each 'go' and while crossing.

The reference shares the camera with both cues, so these are relative timings,
not ground truth. Diagnostic only.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
CACHE = ROOT / '.cache' / 'pose-lite' / 'c3-crops'
MODEL = ROOT / '.cache' / 'pose-lite' / 'pose_landmarker_lite.task'
PAD = 0.25                 # box padding, fraction of box size
VIS = 0.5
LM = dict(nose=0, l_sh=11, r_sh=12, l_hip=23, r_hip=24, l_knee=25, r_knee=26, l_ank=27, r_ank=28)
RANGE_BINS = [(0.5, 1.8), (1.8, 3.0), (3.0, 4.0), (4.0, math.inf)]
GAIT_DEV = 0.15            # ankle separation change, in torso lengths
FACE_MIN = 0.10            # |nose - hip mid| in torso lengths to call a facing
SPEED_ON = 0.15            # c3_score.FALSE_SPEED


# ---------------------------------------------------------------- export (ROS)
def export(run):
    import cv2
    from c1_dataset import load_pair, rgb_on_depth_grid
    ref = json.loads((run / 'c3-reference.json').read_text())['frames']
    out = CACHE / run.name
    out.mkdir(parents=True, exist_ok=True)
    index = []
    for f in ref:
        if not f['boxes'] or f.get('person') is None:
            continue
        b = max(f['boxes'], key=lambda b: b['conf'])['xyxy']
        arrays, row = load_pair(run, f['index'])
        rgb, _ = rgb_on_depth_grid(arrays, row)
        h, w = rgb.shape[:2]
        bw, bh = b[2] - b[0], b[3] - b[1]
        x0, y0 = max(0, int(b[0] - PAD * bw)), max(0, int(b[1] - PAD * bh))
        x1, y1 = min(w, int(b[2] + PAD * bw)), min(h, int(b[3] + PAD * bh))
        name = f'{f["index"]:05d}.jpg'
        # The capture encoding is bgr8, which is what cv2 writes.
        cv2.imwrite(str(out / name), rgb[y0:y1, x0:x1], [cv2.IMWRITE_JPEG_QUALITY, 92])
        index.append(dict(index=f['index'], t_ns=f['depth_stamp_ns'], file=name,
                          offset=[x0, y0], z=f['person']['z']))
    (out / 'index.json').write_text(json.dumps(index) + '\n')
    print(f'{run.name}: {len(index)} crops -> {out}')


# ---------------------------------------------------------------- pose (venv)
def pose(run):
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision
    src = CACHE / run.name
    index = json.loads((src / 'index.json').read_text())
    opts = vision.PoseLandmarkerOptions(base_options=BaseOptions(model_asset_path=str(MODEL)),
                                        running_mode=vision.RunningMode.VIDEO, num_poses=1)
    frames = []
    with vision.PoseLandmarker.create_from_options(opts) as lm:
        last_ms = -1
        for e in index:
            img = cv2.imread(str(src / e['file']))
            h, w = img.shape[:2]
            ms = max(last_ms + 1, e['t_ns'] // 1_000_000)
            last_ms = ms
            res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,
                                               data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), ms)
            pts = None
            if res.pose_landmarks:
                p = res.pose_landmarks[0]
                pts = {k: [e['offset'][0] + p[i].x * w, e['offset'][1] + p[i].y * h,
                           float(min(p[i].visibility or 0, p[i].presence or 0))]
                       for k, i in LM.items()}
            frames.append(dict(t_ns=e['t_ns'], z=e['z'], lm=pts))
    (run / 'c3-pose.json').write_text(json.dumps(dict(
        schema='x3.c3.pose.v1', model=MODEL.name, pad=PAD, frames=frames)) + '\n')
    print(f'{run.name}: pose in {sum(f["lm"] is not None for f in frames)}/{len(frames)} frames')


# ---------------------------------------------------------------- score
def _vis(lm, *keys):
    return lm is not None and all(lm[k][2] >= VIS for k in keys)


def _torso(lm):
    sh = np.mean([lm['l_sh'][:2], lm['r_sh'][:2]], axis=0)
    hp = np.mean([lm['l_hip'][:2], lm['r_hip'][:2]], axis=0)
    return float(np.linalg.norm(sh - hp)), hp


def pose_series(run):
    frames = json.loads((run / 'c3-pose.json').read_text())['frames']
    t = np.array([f['t_ns'] for f in frames]) / 1e9
    gait = np.full(len(t), np.nan)
    face = np.full(len(t), np.nan)
    for i, f in enumerate(frames):
        lm = f['lm']
        if not _vis(lm, 'l_sh', 'r_sh', 'l_hip', 'r_hip'):
            continue
        torso, hip = _torso(lm)
        if torso < 5:
            continue
        if _vis(lm, 'l_ank', 'r_ank'):
            gait[i] = abs(lm['l_ank'][0] - lm['r_ank'][0]) / torso
        if _vis(lm, 'nose'):
            face[i] = (lm['nose'][0] - hip[0]) / torso
    return frames, t, gait, face


def visibility(frames):
    out = {}
    for lo, hi in RANGE_BINS:
        sel = [f for f in frames if lo <= f['z'] < hi]
        if not sel:
            continue
        out[f'{lo:g}-{hi:g}m'] = dict(
            n=len(sel),
            pose=float(np.mean([f['lm'] is not None for f in sel])),
            hips=float(np.mean([_vis(f['lm'], 'l_hip', 'r_hip') for f in sel])),
            shoulders=float(np.mean([_vis(f['lm'], 'l_sh', 'r_sh') for f in sel])),
            ankles=float(np.mean([_vis(f['lm'], 'l_ank', 'r_ank') for f in sel])))
    return out


def go_events(t_ref, speed):
    """Reference 'go' times: >=1 s still (<0.15 m/s) after moving, then moving."""
    t = t_ref / 1e9
    still = speed < SPEED_ON
    events, i = [], 1
    while i < len(t):
        if still[i] and np.nanmax(speed[max(0, i - 15):i], initial=0) > 0.4:
            j = i
            while j < len(t) and (still[j] or np.isnan(speed[j])):
                j += 1
            if j < len(t) and t[j] - t[i] >= 1.0:
                events.append((t[i], t[j]))
            i = j
        i += 1
    return events


def first_onset(t, x, t_go, baseline, window=(-1.0, 2.0)):
    base = x[(t >= baseline[0]) & (t < baseline[1]) & ~np.isnan(x)]
    if base.size < 3:
        return None
    ref = np.median(base)
    idx = np.where((t >= t_go + window[0]) & (t <= t_go + window[1]))[0]
    run = 0
    for k in idx:
        run = run + 1 if (not np.isnan(x[k]) and abs(x[k] - ref) > GAIT_DEV) else 0
        if run == 2:
            return float(t[idx[max(0, np.searchsorted(idx, k) - 1)]] - t_go)
    return None


def arm_onset(eval_run, pos, t_ref, t_go):
    import c3_score as cs
    p = eval_run / 'armC_tracks.csv'
    if not p.exists():
        return None
    by_t = cs.load_arm(p)
    for tt in sorted(k for k in by_t if t_go - 0.3 <= k / 1e9 <= t_go + 2.0):
        k = int(np.argmin(np.abs(t_ref - tt)))
        person = pos[k]
        if person is None:
            continue
        for o in by_t[tt]:
            if math.hypot(o['x'] - person['x'], o['y'] - person['y']) <= cs.MATCH_M and o['speed'] > SPEED_ON:
                return tt / 1e9 - t_go
    return None


def box_u(run):
    ref = json.loads((run / 'c3-reference.json').read_text())['frames']
    t, u = [], []
    for f in ref:
        if f['boxes']:
            b = max(f['boxes'], key=lambda b: b['conf'])['xyxy']
            t.append(f['depth_stamp_ns'] / 1e9)
            u.append((b[0] + b[2]) / 2)
    return np.array(t), np.array(u)


def score(eval_dir, runs):
    import c3_score as cs
    report = {'runs': {}}
    leads, face_pre, face_move = [], [], []
    for run in runs:
        frames, t, gait, face = pose_series(run)
        t_ref, pos, speed = cs.reference_track(run)
        bt, bu = box_u(run)
        r = dict(visibility=visibility(frames), go=[])
        for t_stop, t_go in go_events(t_ref, speed):
            g = first_onset(t, gait, t_go, (t_stop + 0.2, t_go - 0.3))
            c = arm_onset(eval_dir / run.name, pos, t_ref, t_go)
            # Facing before the go vs the direction of the next second's motion.
            pre = face[(t >= t_go - 0.8) & (t < t_go) & ~np.isnan(face)]
            du = np.interp(t_go + 1.0, bt, bu) - np.interp(t_go, bt, bu)
            agree = None
            if pre.size and abs(np.median(pre)) >= FACE_MIN and abs(du) > 10:
                agree = bool(np.sign(np.median(pre)) == np.sign(du))
                face_pre.append(agree)
            r['go'].append(dict(t_go=t_go, pose_onset_s=g, armC_onset_s=c, facing_agrees=agree))
            if g is not None and c is not None:
                leads.append(c - g)
        # While moving laterally: does facing point the way the box moves?
        if cs.scenario_of(run) == 'cross':
            for i in range(len(t)):
                if np.isnan(face[i]) or abs(face[i]) < FACE_MIN:
                    continue
                du = np.interp(t[i] + 0.3, bt, bu) - np.interp(t[i] - 0.3, bt, bu)
                if abs(du) > 15:
                    face_move.append(bool(np.sign(face[i]) == np.sign(du)))
        report['runs'][run.name] = r
    report['summary'] = dict(
        go_events_with_both=len(leads),
        pose_lead_over_armC_s=None if not leads else dict(
            median=float(np.median(leads)), p25=float(np.percentile(leads, 25)),
            p75=float(np.percentile(leads, 75))),
        facing_predicts_go_direction=None if not face_pre else dict(
            n=len(face_pre), agree=float(np.mean(face_pre))),
        facing_matches_crossing_motion=None if not face_move else dict(
            n=len(face_move), agree=float(np.mean(face_move))),
        note='Positive lead = the gait cue fired before arm C crossed 0.15 m/s. '
             'Facing sign conventions may be mirrored; agreement near 0 means '
             'the cue works with the sign flipped, near 0.5 means no signal.')
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)
    for m in ('export', 'pose'):
        s = sub.add_parser(m)
        s.add_argument('runs', type=Path, nargs='+')
    s = sub.add_parser('score')
    s.add_argument('eval_dir', type=Path)
    s.add_argument('runs', type=Path, nargs='+')
    s.add_argument('--output', type=Path)
    args = ap.parse_args()
    if args.mode == 'export':
        for r in args.runs:
            export(r)
    elif args.mode == 'pose':
        for r in args.runs:
            pose(r)
    else:
        rep = score(args.eval_dir, args.runs)
        if args.output:
            args.output.write_text(json.dumps(rep, indent=1) + '\n')
        print(json.dumps(rep['summary'], indent=1))
        for name, r in rep['runs'].items():
            print(name, json.dumps(r['visibility']))


if __name__ == '__main__':
    main()
