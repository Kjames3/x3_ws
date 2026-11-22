#!/usr/bin/env python3
"""Do synthetic seated people and furniture help the lidar person classifier?  (2026-11-22)

Step 0 of notes/plans/2026-11-21-isaac-sim-lidar-people.md.  Two subcommands.

  build  Insert people (and furniture) into REAL Livox frames with synthetic_people.py and
         write one row per candidate the extractor yields, plus one row per synthetic person.

    python3 src/tools/perception/synthetic_seated_experiment.py build --seed 0 \
        --own artifacts/lidar-person-classifier-2026-11-01/own_1121.npz --data-root ~/x3_ws/artifacts \
        -o artifacts/lidar-person-classifier-2026-11-01/synth_seated_s0.npz --jobs 28

  train  Leave-one-run-out on the REAL runs only, with synthetic rows added to every training
         fold (never the held-out run's own backgrounds), arms a..d, 3 generator seeds.

    python3 src/tools/perception/synthetic_seated_experiment.py train \
        --own .../own_1121.npz --synth .../synth_seated_s0.npz .../synth_seated_s1.npz .../synth_seated_s2.npz \
        -o artifacts/lidar-person-classifier-2026-11-01/results_synth_seated_2026-11-22.txt

Synthetic rows are labelled from the point labels of the insertion, not from the extractor's
centre match (which would call a mostly-desk candidate a person and teach "furniture = person",
the SiT failure in lidar_person_classifier.py):

  y = 1   the candidate's main person has >= MIN_PERSON_POINTS points in it and they are >= PURITY of
          the candidate's points, where a seated person's own chair/couch counts as neutral
          (real seated labels are person-plus-chair blobs);
  y = 0   < 5% of its points are a person's and >= 30% are synthetic furniture (empty chairs and desks,
          a shelf beside a person, the same scene with the person removed);
  y = -1  anything between; left out of training.

Synthetic clutter that is only real background is NOT used as a negative: the real runs already
carry those, and a background frame may hold an unlabelled real person.

GO / NO-GO RULE (fixed before the first run, see RULE below).
"""
import argparse
import glob
import math
import multiprocessing as mp
import os
import pathlib
import sys

import numpy as np

_HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parents[1] / 'yahboomcar_bringup'))

import synthetic_people as sp  # noqa: E402
from yahboomcar_bringup import cluster_features as cf  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

MIN_PERSON_POINTS = 10
PURITY = 0.6
NEG_PERSON_SHARE, NEG_FURN_SHARE = 0.05, 0.30
SEAT_KINDS = ('chair', 'couch')
NEW_KINDS = ('close', 'seated_desk', 'seated_chair', 'seated_couch', 'stand_furn')
EXCLUDE_RUNS = {'trial', 'trial-lab', 'trial-lab2', 'apt-desk2-trial', 'apt-smoke-r1', 'bias-check-r1'}

# Scenes per generator seed: (kind, count, P(abut)).  std_A / std_B are two independent halves of the
# EXISTING generator (standing/walking groups, 0.8-12 m); b uses A, b_big uses A+B (4.4k rows, about half of
# c_all's 8.7k, so c_all - b_big is not volume-controlled), c_all is A plus the new kinds.
SCENES = (('std_A', 3000, 0.0), ('std_B', 3000, 0.0), ('close', 1500, 0.0),
          ('seated_desk', 2000, 0.4), ('seated_chair', 1000, 0.4), ('seated_couch', 1000, 0.4),
          ('stand_furn', 1500, 0.4))

RULE = """GO / NO-GO RULE (written before the first run)
  Content helps  <=>  on the pooled seated runs, AP(c_all) - AP(b_big) is >= +0.02, its run-bootstrap 95% CI
                      lies above 0, and it exceeds the across-generator-seed range of AP(c_all);
                      and nothing regresses: overall AP(c_all) - AP(a) >= -0.005 and false alarms per frame on
                      the person-free runs rise by < 25%.
  Fidelity gap   <=>  among synthetic seated people the extractor still finds, the median of n, width, depth,
                      height, z_hi, top_fill and z_std differs from the real seated (label 2) candidates by
                      more than 10% in at least two of the three range bands 0-2, 2-4, 4-6 m.
  GO for the Isaac spike  <=>  content helps AND a fidelity gap remains (a better renderer could add something).
  NO-GO  <=>  content does not help (the bottleneck is the extractor, the C3 labels or the data volume, not
              the synthetic content), or content helps and there is no fidelity gap (step 0 already captured it)."""

FRAMES = []      # (xyz, tsec, real_people, run), filled before the pool forks


# ---------------------------------------------------------------- build

def _view(p):
    """Which side of a person the sensor sees: front, side or back."""
    to_sensor = math.atan2(-p['y'], -p['x'])
    d = abs((p['yaw'] - to_sensor + math.pi) % (2 * math.pi) - math.pi)
    return 'front' if d < math.pi / 3 else ('back' if d > 2 * math.pi / 3 else 'side')


def candidate_rows(out, pid, people, pieces, cands):
    """One dict per candidate that holds person points or mostly synthetic furniture."""
    x, y, z = out[:, 0], out[:, 1], out[:, 2]
    seat_ids = {-2 - j for j, f in enumerate(pieces) if f['kind'] in SEAT_KINDS}
    rows = []
    for c in cands:
        m = cf.crop(x, y, z, c)
        p = pid[m]
        n = len(p)
        if n == 0:
            continue
        cnt = np.bincount(p[p >= 0], minlength=len(people))[:len(people)] if len(people) else np.zeros(1, int)
        main = int(cnt.argmax())
        main_pts, p_pts = int(cnt[main]), int(cnt.sum())
        f_share = float((p <= -2).mean())
        if p_pts / n < NEG_PERSON_SHARE and f_share < NEG_FURN_SHARE:
            continue
        seated = bool(len(people) and people[main].get('seated'))
        own_seat = int(np.isin(p, list(seat_ids)).sum()) if seated else 0
        rows.append(dict(X=np.asarray(cf.features(x[m], y[m], z[m]), dtype=np.float32),
                         range=float(math.hypot(c[0], c[1])), n=n, main_pts=main_pts, p_share=p_pts / n,
                         f_share=f_share, purity_raw=main_pts / n, purity=main_pts / max(1, n - own_seat),
                         seated=seated, n_people=int((cnt >= 5).sum()), main=main))
    return rows


def run_scene(job):
    kind, fi, seed, abut = job
    xyz, tsec, real_people, bg = FRAMES[fi]
    rng = np.random.default_rng(seed)
    occupied, open_bearing = sp.frame_context(xyz, rng)
    people, pieces, meta = [], [], {}
    if kind.startswith('std'):
        people = sp.sample_group(rng, occupied, real_people, open_bearing)
    elif kind == 'close':
        people = sp.sample_close_group(rng, occupied, real_people, open_bearing)
    else:
        res = sp.sample_furniture_scene(rng, kind, occupied, real_people, open_bearing, abut=abut)
        if res:
            people, pieces, meta = res
    if not people:
        return [], []
    tag = dict(kind=kind, bg=bg, abut=bool(abut and pieces), seed=seed)
    cand_rows, person_rows = [], []

    def extract_once(out, pid, ppl, pcs):
        cands = pc.extract(out[:, 0], out[:, 1], out[:, 2], origin=(0.0, 0.0), range_max=sp.RANGE_MAX_M)
        return cands, candidate_rows(out, pid, ppl, pcs, cands)

    out, pid = sp.insert(xyz, tsec, people, rng, pieces)
    cands, rows = extract_once(out, pid, people, pieces)
    res = sp.score(out, pid, people, lambda *a, **k: cands)
    # the same people with no furniture around them (seated people then float): what the furniture costs
    if pieces:
        out_b, pid_b = sp.insert(xyz, tsec, people, rng)
        res_bare = sp.score(out_b, pid_b, people, lambda *a, **k: pc.extract(
            out_b[:, 0], out_b[:, 1], out_b[:, 2], origin=(0.0, 0.0), range_max=sp.RANGE_MAX_M))
    else:
        res_bare = res
    P = np.array([[p['x'], p['y']] for p in people])
    for k, p in enumerate(people):
        others = np.delete(P, k, axis=0)
        person_rows.append(dict(
            tag, range=float(math.hypot(p['x'], p['y'])), seated=bool(p.get('seated')), outcome=res[k][0],
            outcome_bare=res_bare[k][0], visible=res[k][2], view=_view(p),
            furn=meta.get('furn', ''), occluding=bool(meta.get('occluding', False)), walking=p['speed'] > 0,
            nn=float(np.hypot(*(others - P[k]).T).min()) if len(others) else 99.0, X=res[k][1]))
    for r in rows:
        cand_rows.append(dict(tag, **r, empty=False, seated_scene=bool(people[0].get('seated'))))
    if pieces:    # the same furniture without anyone: the empty chair / desk / shelf negatives
        out_e, pid_e = sp.insert(xyz, tsec, [], rng, pieces)
        _, rows_e = extract_once(out_e, pid_e, [], pieces)
        for r in rows_e:
            cand_rows.append(dict(tag, **r, empty=True, seated_scene=bool(people[0].get('seated'))))
    return cand_rows, person_rows


def _load(args):
    run_dir, every, own = args
    return pathlib.Path(run_dir).name, sp.load_frames(run_dir, every, own)


def build(a):
    own = dict(np.load(a.own, allow_pickle=True))
    runs = sorted(set(own['run']))
    dirs = {}
    for d in glob.glob(os.path.join(os.path.expanduser(a.data_root), 'person-lidar-2026-1*', '*', '')):
        name = pathlib.Path(d).name
        if name in runs and name not in EXCLUDE_RUNS:
            dirs[name] = d
    missing = [r for r in runs if r not in dirs]
    print(f'{len(dirs)} runs with bags, {len(missing)} without {missing}')
    global FRAMES
    with mp.Pool(a.jobs) as pool:
        for name, fr in pool.imap_unordered(_load, [(d, a.every, own) for d in dirs.values()]):
            FRAMES += [(x, t, rp, name) for x, t, rp in fr]
    print(f'{len(FRAMES)} background frames from {len(set(f[3] for f in FRAMES))} runs')
    rng = np.random.default_rng(a.seed)
    jobs = []
    for kind, count, p_abut in SCENES:
        for _ in range(max(1, int(count * a.scale))):
            jobs.append((kind, int(rng.integers(len(FRAMES))), int(rng.integers(1 << 40)), bool(rng.random() < p_abut)))
    cand, person = [], []
    with mp.Pool(a.jobs) as pool:
        for c, p in pool.imap_unordered(run_scene, jobs, chunksize=8):
            cand += c
            person += p
    print(f'{len(jobs)} scenes -> {len(cand)} candidate rows, {len(person)} synthetic people')
    out = {}
    for name, rows in (('c', cand), ('p', person)):
        for k in rows[0]:
            v = [r[k] for r in rows]
            out[f'{name}_{k}'] = np.array(v) if k != 'X' else np.stack(v)
    out['feature_names'] = np.array(cf.NAMES)
    np.savez_compressed(a.out, **out)
    print(f'wrote {a.out}')


# ---------------------------------------------------------------- train / evaluate

def synth_labels(d, purity=PURITY):
    """Training label per synthetic candidate row: 1 / 0 / -1 (see the module docstring)."""
    y = np.full(len(d['c_kind']), -1)
    pos = (d['c_main_pts'] >= MIN_PERSON_POINTS) & (d['c_purity'] >= purity)
    neg = (d['c_p_share'] < NEG_PERSON_SHARE) & (d['c_f_share'] >= NEG_FURN_SHARE)
    y[neg] = 0
    y[pos] = 1
    return y


def arm_rows(d, arm, purity=PURITY):
    """Boolean mask of the synthetic rows an arm trains on, and their labels."""
    y = synth_labels(d, purity)
    kind = d['c_kind']
    std_a, std_b = kind == 'std_A', kind == 'std_B'
    new = np.isin(kind, NEW_KINDS)
    if arm in ('b', 'b_w25'):
        m = std_a
    elif arm == 'b_big':
        m = std_a | std_b
    elif arm in ('c_all', 'c_loose', 'c_w25'):
        m = std_a | new
    elif arm == 'c_new':
        m = new
    elif arm == 'c_pos':        # new content, positives only (no furniture negatives)
        m = std_a | (new & (y == 1))
    elif arm == 'c_neg':        # std positives + the new content's furniture negatives only
        m = std_a | (new & (y == 0))
    elif arm == 'd':
        m = std_a | std_b | new
    else:
        raise ValueError(arm)
    return m & (y >= 0), y


ARMS = ('a', 'b', 'b_big', 'c_all', 'c_new', 'c_pos', 'c_neg', 'c_loose', 'b_w25', 'c_w25', 'd')


def _fit_fold(job):
    from sklearn.ensemble import HistGradientBoostingClassifier
    Xr, yr, Xs, ys, Xte, seed, w_syn = job
    if len(Xs):
        X, y = np.vstack([Xr, Xs]), np.r_[yr, ys]
        w = np.r_[np.ones(len(Xr)), np.full(len(Xs), w_syn)]
    else:
        X, y, w = Xr, yr, None
    clf = HistGradientBoostingClassifier(max_iter=300, learning_rate=0.08, max_leaf_nodes=31,
                                         l2_regularization=1.0, class_weight='balanced', random_state=seed)
    return clf.fit(X, y, sample_weight=w).predict_proba(Xte)[:, 1]


def loro(own, synths, arm, seed, pool):
    """Leave-one-real-run-out predictions for every real row (nan where the run has no training data)."""
    y, use, run = own['y_map'], own['use'], own['run']
    S = synths[seed] if synths else None
    w_syn = 0.25 if arm.endswith('_w25') else 1.0
    sm, sy = (arm_rows(S, arm, 0.35 if arm == 'c_loose' else PURITY) if S is not None and arm != 'a' else (None, None))
    jobs, runs = [], np.unique(run)
    for r in runs:
        tr = use & (run != r)
        if arm == 'd':
            keep = sm & (S['c_bg'] != r)
            jobs.append((own['X'][:0], y[:0], S['c_X'][keep], sy[keep], own['X'][run == r], seed, w_syn))
            continue
        Xs, ys = (S['c_X'][sm & (S['c_bg'] != r)], sy[sm & (S['c_bg'] != r)]) if sm is not None else (own['X'][:0], y[:0])
        jobs.append((own['X'][tr], y[tr], Xs, ys, own['X'][run == r], seed, w_syn))
    p = np.full(len(y), np.nan)
    for r, pr in zip(runs, pool.map(_fit_fold, jobs)):
        p[run == r] = pr
    return p


def ap(y, p):
    from sklearn.metrics import average_precision_score
    return float(average_precision_score(y, p)) if 0 < y.sum() < len(y) else float('nan')


SUBSETS = {
    'seated runs (16)': ['apt-desk2-drive-r1', 'apt-desk2-drive-r2', 'apt-desk2-empty-park15-r1', 'apt-desk2-park07-r1',
                             'apt-desk2-park15-r1', 'apt-desk2-park30-r1', 'apt-seated-chair-r1', 'apt-seated-chair-r2',
                             'apt-seated-couch-r1', 'apt-seated-couch-r2', 'apt-seated-desk-r1', 'furniture-r1',
                             'goal-on-r4', 'seated-r1', 'seated-r2', 'seated-r3'],
    '  lab desk 11-20': ['seated-r1', 'seated-r2', 'seated-r3'],
    '  apt desk2 (7 runs)': ['apt-desk2-drive-r1', 'apt-desk2-drive-r2', 'apt-desk2-empty-park15-r1', 'apt-desk2-park07-r1',
                            'apt-desk2-park15-r1', 'apt-desk2-park30-r1', 'apt-desk2-empty-drive-r1'],
    '  apt chair/couch/desk': ['apt-seated-chair-r1', 'apt-seated-chair-r2', 'apt-seated-couch-r1', 'apt-seated-couch-r2',
                               'apt-seated-desk-r1'],
    '  furniture-r1 + goal-on-r4': ['furniture-r1', 'goal-on-r4'],
    'crowd (busy, group)': ['busy-r1', 'busy-r2', 'group-r1'],
    'close runs (apt-close, apt-occluded, walkup)': ['apt-close-r1', 'apt-occluded-r1', 'walkup-r1', 'walkup-r2', 'walkup-r3'],
}
SEATED_RUNS = SUBSETS['seated runs (16)']
NEG_RUNS = ['empty-r1', 'goal-on-r1', 'apt-desk2-empty-drive-r1', 'walk-r1']


def subset_mask(own, runs):
    return np.isin(own['run'], runs) & own['use']


def metrics(own, p):
    """Dict of the report's numbers for one prediction vector over the real rows."""
    y, use, run, rng = own['y_map'], own['use'], own['run'], own['X'][:, 0]
    out = {'all': ap(y[use], p[use])}
    for lo, hi in ((0, 2), (2, 4), (4, 6)):
        m = use & (rng >= lo) & (rng < hi)
        out[f'{lo}-{hi}m'] = ap(y[m], p[m])
    m = use & (rng < 1.5)
    out['<1.5m'] = ap(y[m], p[m])
    for name, runs in SUBSETS.items():
        m = subset_mask(own, runs)
        out[name] = ap(y[m], p[m])
        seat = m & own['seat']
        out[name + ' | seated recall'] = float((p[seat] >= 0.5).mean()) if seat.any() else float('nan')
        out[name + ' | precision'] = float(((p[m] >= 0.5) & (y[m] == 1)).sum() / max(1, (p[m] >= 0.5).sum()))
    for r in NEG_RUNS:
        m = (run == r) & use & (y == 0)
        nf = len(np.unique(own['t'][run == r])) or 1
        out[f'FP/frame {r}'] = float((p[m] >= 0.5).sum() / nf)
    return out


def bootstrap(own, p_new, p_old, runs, n=2000, seed=0):
    """Paired bootstrap over runs: pooled AP(new) - AP(old) on the rows of `runs`."""
    rng = np.random.default_rng(seed)
    y, use, run = own['y_map'], own['use'], own['run']
    idx = {r: np.flatnonzero((run == r) & use) for r in runs}
    deltas = []
    for _ in range(n):
        pick = rng.choice(runs, size=len(runs))
        ii = np.concatenate([idx[r] for r in pick])
        if len(ii) == 0 or y[ii].sum() == 0 or y[ii].sum() == len(ii):
            continue
        deltas.append(ap(y[ii], p_new[ii]) - ap(y[ii], p_old[ii]))
    d = np.array(deltas)
    return float(d.mean()), float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))


def extractor_report(synths):
    """Extractor outcomes for synthetic seated people, by scene kind and range, and the feature comparison."""
    lines = []
    P = {k: np.concatenate([s[k] for s in synths]) for k in synths[0] if k.startswith('p_')}
    scored = P['p_outcome'] != 'hidden'
    cover = lambda o: np.isin(o, ('found', 'merged_person', 'merged_other'))

    def cell(m):
        m = m & scored
        if m.sum() < 15:
            return f'{"--":>26s}'
        o = P['p_outcome'][m]
        ob = P['p_outcome_bare'][m]
        return (f'{100 * (o == "found").mean():3.0f}% f {100 * cover(o).mean():3.0f}% cov '
                f'(bare {100 * (ob == "found").mean():3.0f}%) n{m.sum():5d}')
    lines.append('found = a candidate within 0.35 m of the person; cov = some candidate box covers them (found + merged);')
    lines.append('bare = the same people with the furniture removed (seated people then float: it isolates what furniture costs)')
    lines.append('hidden (< 10 points) people are not scored: ' + ', '.join(
        f'{k} {100 * (P["p_outcome"][P["p_kind"] == k] == "hidden").mean():.0f}%' for k in NEW_KINDS + ('std_A',)))
    bands = ((0.4, 1.5), (1.5, 3), (3, 4.5), (4.5, 6.5))
    for title, sel in (('by scene kind and range', None),):
        lines.append(f'\n  {"kind":22s}|' + '|'.join(f'{f"{lo}-{hi} m":>26s}' for lo, hi in bands) + f'|{"all":>26s}')
        for kind in NEW_KINDS + ('std_A',):
            mk = P['p_kind'] == kind
            lines.append(f'  {kind:22s}|' + '|'.join(cell(mk & (P['p_range'] >= lo) & (P['p_range'] < hi)) for lo, hi in bands)
                         + '|' + cell(mk))
    sm = P['p_seated']
    lines.append('\n  seated people only:')
    for name, m in (('back view', P['p_view'] == 'back'), ('side view', P['p_view'] == 'side'), ('front view', P['p_view'] == 'front'),
                    ('furniture abuts real clutter', P['p_abut']), ('furniture on free floor', ~P['p_abut'])):
        lines.append(f'  {name:26s}|' + '|'.join(cell(sm & m & (P['p_range'] >= lo) & (P['p_range'] < hi)) for lo, hi in bands)
                     + '|' + cell(sm & m))
    lines.append('\n  standing/walking beside furniture (stand_furn):')
    for f in ('bookcase', 'partition', 'desk', 'couch'):
        for occ in (False, True):
            m = (P['p_kind'] == 'stand_furn') & (P['p_furn'] == f) & (P['p_occluding'] == occ)
            lines.append(f'  {f + (" (occluding)" if occ else " (beside)"):26s}|' + cell(m))
    return lines, P


def feature_comparison(P, own):
    names = [str(n) for n in own['feature_names']]
    keys = [k for k in ('n', 'width', 'depth', 'height', 'z_hi', 'top_fill', 'z_std') if k in names]
    real = own['X'][own['seat']]
    found = (P['p_outcome'] == 'found') & P['p_seated']
    syn = P['p_X'][found]
    lines = [f'  {"range":>7s} {"count":>11s} |' + ' |'.join(f'{k:>16s}' for k in keys), '  (median real label-2 / synthetic found seated, % difference)']
    gaps = []
    for lo, hi in ((0, 2), (2, 4), (4, 6)):
        a = real[(real[:, 0] >= lo) & (real[:, 0] < hi)]
        b = syn[(syn[:, 0] >= lo) & (syn[:, 0] < hi)]
        if len(a) < 20 or len(b) < 20:
            lines.append(f'  {lo}-{hi} m  {len(a)}/{len(b)} too few')
            continue
        cells, bad = [], 0
        for k in keys:
            ma, mb = float(np.median(a[:, names.index(k)])), float(np.median(b[:, names.index(k)]))
            diff = 100 * (mb - ma) / ma if ma else float('nan')
            bad += abs(diff) > 10
            cells.append(f'{ma:6.2f}/{mb:<6.2f}{diff:+4.0f}%')
        gaps.append(bad >= 1)
        lines.append(f'  {lo}-{hi} m {len(a):5d}/{len(b):<5d} |' + ' |'.join(f'{c:>16s}' for c in cells))
    return lines, gaps


def domain_gap(own, synths):
    """AUC of a classifier separating real person candidates from synthetic ones (same range band).
    The 'real vs real' rows are the floor: two arbitrary halves of the real runs (grouped by run)."""
    from sklearn.ensemble import HistGradientBoostingClassifier
    from sklearn.metrics import roc_auc_score
    from sklearn.model_selection import GroupKFold, StratifiedKFold, cross_val_predict
    S = synths[0]
    ys = synth_labels(S)
    rng = np.random.default_rng(0)
    X_, y_, run_ = own['X'], own['y_map'], own['run']
    out = []
    clf = lambda: HistGradientBoostingClassifier(max_iter=100, random_state=0)

    def auc(Xa, Xb, groups=None):
        n = min(len(Xa), len(Xb), 6000)
        Xa, Xb = Xa[rng.choice(len(Xa), n, replace=False)], Xb[rng.choice(len(Xb), n, replace=False)]
        X, y = np.vstack([Xa, Xb]), np.r_[np.zeros(n), np.ones(n)]
        p = cross_val_predict(clf(), X, y, cv=StratifiedKFold(3, shuffle=True, random_state=0), method='predict_proba')[:, 1]
        return roc_auc_score(y, p), n

    for title, real_m, syn_m in (
            ('standing/walking: real y=1 vs synthetic std_A', (y_ == 1) & ~own['seat'], (S['c_kind'] == 'std_A') & (ys == 1)),
            ('seated: real label 2 vs synthetic seated y=1', own['seat'], (ys == 1) & S['c_seated'])):
        for lo, hi in ((0, 2), (2, 4)):
            a = X_[real_m & (X_[:, 0] >= lo) & (X_[:, 0] < hi)]
            b = S['c_X'][syn_m & (S['c_range'] >= lo) & (S['c_range'] < hi)]
            if len(a) > 50 and len(b) > 50:
                v, n = auc(a, b)
                out.append(f'  {title:48s} {lo}-{hi} m: AUC {v:.3f}  ({n} vs {n} rows)')
    # floor: real positives of half the runs vs the other half; a classifier that has to carry over to unseen runs
    pos = (y_ == 1) | own['seat']
    uruns = np.unique(run_)
    half = np.isin(run_, rng.permutation(uruns)[:len(uruns) // 2])
    m = pos & (X_[:, 0] < 4)
    idx = np.flatnonzero(m)
    idx = rng.choice(idx, min(len(idx), 12000), replace=False)
    p = cross_val_predict(clf(), X_[idx], half[idx].astype(int), groups=run_[idx], cv=GroupKFold(3), method='predict_proba')[:, 1]
    out.append(f'  {"floor: real positives, half the runs vs the other half":48s} 0-4 m: AUC {roc_auc_score(half[idx], p):.3f}  '
               f'(rows from unseen runs; 12000 rows)')
    return '\n'.join(out)


def train(a):
    own = dict(np.load(a.own, allow_pickle=True))
    y = own['y'].copy()
    own['seat'] = y == 2
    y[y == 2] = 1
    own['y_map'] = y
    own['use'] = (y == 0) | (y == 1)
    own['X'] = own['X'].astype(np.float32)
    synths = [np.load(s, allow_pickle=True) for s in a.synth]
    synths = [dict(s) for s in synths]
    for s in synths:
        s['c_X'] = s['c_X'].astype(np.float32)
    seeds = list(range(len(synths)))
    print(RULE)
    print(f'\nreal: {int(own["use"].sum())} usable rows, {len(np.unique(own["run"]))} runs; synthetic: {len(synths)} generator seeds')
    for i, s in enumerate(synths):
        kind = s['c_kind']
        yy = synth_labels(s)
        print(f'  seed {i}: ' + ', '.join(f'{k} {int((kind == k).sum())} rows (y1 {int(((kind == k) & (yy == 1)).sum())}, '
                                          f'y0 {int(((kind == k) & (yy == 0)).sum())})' for k in ('std_A', 'std_B') + NEW_KINDS))
    # purity histogram of synthetic seated person candidates
    s = synths[0]
    m = s['c_seated'] & ~s['c_empty'] & (s['c_main_pts'] >= MIN_PERSON_POINTS)
    h = np.histogram(s['c_purity'][m], bins=[0, .2, .4, .6, .8, 1.01])[0]
    h2 = np.histogram(s['c_purity_raw'][m], bins=[0, .2, .4, .6, .8, 1.01])[0]
    print('\nPurity of candidates holding a seated person (>= 10 of their points; seed 0), bins 0-.2 .2-.4 .4-.6 .6-.8 .8-1:')
    print(f'  neutral own chair {h.tolist()}   raw {h2.tolist()}   -> y=1 needs >= {PURITY}')

    from concurrent.futures import ProcessPoolExecutor
    P = {}
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        for arm in ARMS:
            for sd in (seeds if arm != 'a' else seeds):
                P[(arm, sd)] = loro(own, synths, arm, sd, pool)
                print(f'  done {arm} seed {sd}  AP all {ap(own["y_map"][own["use"]], P[(arm, sd)][own["use"]]):.4f}', flush=True)
    avg = {arm: np.mean([P[(arm, sd)] for sd in seeds], axis=0) for arm in ARMS}
    M = {(arm, sd): metrics(own, P[(arm, sd)]) for arm in ARMS for sd in seeds}
    Mavg = {arm: metrics(own, avg[arm]) for arm in ARMS}

    print('\n== Leave-one-run-out AP on the REAL runs only (mean over generator seeds; [min..max] across seeds) ==')
    print('arms: a real only | b real+existing synthetic (std A) | b_big same, std A+B (about half of c_all\'s rows: NOT volume-matched) |')
    print('      c_all real+std A+new | c_new real+new only | c_pos new positives only | c_neg new negatives only |')
    print('      c_loose c_all with purity >= 0.35 | b_w25 / c_w25 b / c_all with every synthetic row weighted 0.25 | d synthetic only')
    for arm in ARMS:
        print(f'  {arm:8s} synthetic rows used: ' + (', '.join(
            str(int(arm_rows(s, arm, 0.35 if arm == 'c_loose' else PURITY)[0].sum())) for s in synths) if arm != 'a' else '0'))
    cols = ['all', '0-2m', '2-4m', '4-6m', '<1.5m'] + list(SUBSETS)
    print(f'\n  {"arm":8s}|' + '|'.join(f'{c[:22]:>22s}' for c in cols))
    for arm in ARMS:
        cells = []
        for c in cols:
            v = [M[(arm, sd)][c] for sd in seeds]
            cells.append(f'{np.nanmean(v):.3f} [{np.nanmin(v):.3f}..{np.nanmax(v):.3f}]')
        print(f'  {arm:8s}|' + '|'.join(f'{c:>22s}' for c in cells))
    print('\n  AP of the seed-averaged probabilities (what the bootstrap uses):')
    print(f'  {"arm":8s}|' + '|'.join(f'{c[:22]:>22s}' for c in cols))
    for arm in ARMS:
        print(f'  {arm:8s}|' + '|'.join(f'{Mavg[arm][c]:>22.3f}' for c in cols))

    print('\n== Seated recall @0.5 (label-2 candidates scored >= 0.5) and precision @0.5, seed-averaged probabilities ==')
    sub = list(SUBSETS)
    print(f'  {"arm":8s}|' + '|'.join(f'{c[:22]:>22s}' for c in sub))
    for arm in ARMS:
        print(f'  {arm:8s}|' + '|'.join(f'{Mavg[arm][c + " | seated recall"]:>10.2f}/{Mavg[arm][c + " | precision"]:<11.2f}' for c in sub))
    print('  (recall/precision; precision counts every person-labelled candidate of the subset)')

    print('\n== False alarms per frame at 0.5 on the labelled negatives of person-free / low-person runs ==')
    print(f'  {"arm":8s}|' + '|'.join(f'{r:>26s}' for r in NEG_RUNS))
    for arm in ARMS:
        print(f'  {arm:8s}|' + '|'.join(f'{Mavg[arm]["FP/frame " + r]:>26.4f}' for r in NEG_RUNS))

    print('\n== Per-run AP on the seated runs (seed-averaged probabilities) ==')
    y_, use_, run_ = own['y_map'], own['use'], own['run']
    print(f'  {"run":28s}' + ''.join(f'{arm:>9s}' for arm in ('a', 'b_big', 'c_all', 'c_neg', 'c_pos')) + '   n pos / neg   seated recall a -> c_all')
    for r in SEATED_RUNS + ['apt-desk2-empty-drive-r1']:
        m = (run_ == r) & use_
        if not m.any():
            continue
        vals = ''.join(f'{ap(y_[m], avg[arm][m]):9.3f}' for arm in ('a', 'b_big', 'c_all', 'c_neg', 'c_pos'))
        sm = (run_ == r) & own['seat']
        rec = (f'{(avg["a"][sm] >= .5).mean():.2f} -> {(avg["c_all"][sm] >= .5).mean():.2f}' if sm.any() else '   --')
        print(f'  {r:28s}{vals}   {int((y_[m] == 1).sum()):5d}/{int((y_[m] == 0).sum()):<5d}   {rec}')

    print('\n== Paired bootstrap over runs (2000 resamples): mean AP difference [95% CI] ==')
    pairs = (('b_big', 'a'), ('c_all', 'a'), ('c_all', 'b_big'), ('c_all', 'b'), ('c_neg', 'b_big'), ('c_pos', 'b_big'), ('c_new', 'a'))
    boot = {}
    for name, runs in (('seated runs (16)', SEATED_RUNS), ('crowd (busy, group)', SUBSETS['crowd (busy, group)']),
                       ('close runs', SUBSETS['close runs (apt-close, apt-occluded, walkup)']),
                       ('all real runs', [r for r in np.unique(run_)])):
        for new, old in pairs:
            boot[(name, new, old)] = bootstrap(own, avg[new], avg[old], runs, n=a.boot)
            m_, lo_, hi_ = boot[(name, new, old)]
            print(f'  {name:24s} {new:7s} - {old:6s}: {m_:+.4f} [{lo_:+.4f}, {hi_:+.4f}]')

    print('\n  exploratory, single subsets (few runs each; not part of the decision rule):')
    for name in ('  lab desk 11-20', '  apt desk2 (7 runs)', '  apt chair/couch/desk', '  furniture-r1 + goal-on-r4'):
        for new, old in (('c_all', 'a'), ('c_all', 'b_big'), ('c_pos', 'a'), ('c_neg', 'a')):
            m_, lo_, hi_ = bootstrap(own, avg[new], avg[old], SUBSETS[name], n=a.boot)
            print(f'  {name.strip():28s} {new:7s} - {old:6s}: {m_:+.4f} [{lo_:+.4f}, {hi_:+.4f}]')

    print('\n== Why furniture-r1 collapses: real seated candidates vs synthetic ones (median) ==')
    names_ = [str(n) for n in own['feature_names']]
    ks = ['range', 'n', 'height', 'z_hi', 'width', 'depth', 'top_fill']
    ix = [names_.index(k) for k in ks]
    print(f'  {"":44s}' + ''.join(f'{k:>10s}' for k in ks) + '      rows')
    S0 = synths[0]
    y0 = synth_labels(S0)
    close = S0['c_range'] < 1.5
    for name, X in (('real furniture-r1 label 2', own['X'][(own['run'] == 'furniture-r1') & own['seat']]),
                    ('real label 2, all runs, < 1.5 m', own['X'][own['seat'] & (own['X'][:, 0] < 1.5)]),
                    ('synthetic seated person (y=1), < 1.5 m', S0['c_X'][(y0 == 1) & S0['c_seated'] & close]),
                    ('synthetic furniture only (y=0, empty), < 1.5 m', S0['c_X'][(y0 == 0) & S0['c_empty'] & close]),
                    ('synthetic furniture beside a person (y=0), < 1.5 m', S0['c_X'][(y0 == 0) & ~S0['c_empty'] & close])):
        if len(X):
            print(f'  {name:44s}' + ''.join(f'{np.median(X[:, i]):10.2f}' for i in ix) + f'{len(X):10d}')

    print('\n== Domain gap: can a classifier tell synthetic person candidates from real ones?  (3-fold CV AUC; 0.5 = cannot, 1 = trivially) ==')
    print(domain_gap(own, synths))

    print('\n== Extractor on synthetic people (the same candidate extractor as the live system) ==')
    lines, Pp = extractor_report(synths)
    print('\n'.join(lines))
    print('\n== Fidelity: synthetic seated people the extractor found vs real label-2 candidates ==')
    lines, gaps = feature_comparison(Pp, own)
    print('\n'.join(lines))

    # ---- decision (rule above)
    sd_c = [M[('c_all', sd)]['seated runs (16)'] for sd in seeds]
    d_seated = Mavg['c_all']['seated runs (16)'] - Mavg['b_big']['seated runs (16)']
    m_, lo_, hi_ = boot[('seated runs (16)', 'c_all', 'b_big')]
    seed_range = float(np.nanmax(sd_c) - np.nanmin(sd_c))
    helps = (d_seated >= 0.02 and lo_ > 0 and d_seated > seed_range)
    d_all = Mavg['c_all']['all'] - Mavg['a']['all']
    fp_up = [Mavg['c_all'][f'FP/frame {r}'] / max(1e-9, Mavg['a'][f'FP/frame {r}']) for r in NEG_RUNS if Mavg['a'][f'FP/frame {r}'] > 0.01]
    nothing_regresses = d_all >= -0.005 and (not fp_up or max(fp_up) < 1.25)
    fidelity_gap = sum(gaps) >= 2
    print('\n== Decision ==')
    print(f'  seated pooled AP: c_all {Mavg["c_all"]["seated runs (16)"]:.3f}  b_big {Mavg["b_big"]["seated runs (16)"]:.3f}'
          f'  delta {d_seated:+.3f}, bootstrap {m_:+.3f} [{lo_:+.3f}, {hi_:+.3f}], c_all seed range {seed_range:.3f}')
    print(f'  overall AP c_all - a {d_all:+.4f}; FP/frame ratios c_all/a on person-free runs {[round(x, 2) for x in fp_up]}')
    print(f'  content helps: {helps};  nothing regresses: {nothing_regresses};  fidelity gap in {sum(gaps)} of {len(gaps)} bands: {fidelity_gap}')
    verdict = 'GO' if (helps and nothing_regresses and fidelity_gap) else 'NO-GO'
    print(f'  Isaac Sim spike: {verdict}')
    if a.out_npz:
        np.savez_compressed(a.out_npz, **{f'p_{arm}_{sd}': P[(arm, sd)] for arm in ARMS for sd in seeds}, run=own['run'])


def main():
    ap_ = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap_.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build')
    b.add_argument('--own', required=True)
    b.add_argument('--data-root', default='~/x3_ws/artifacts')
    b.add_argument('--seed', type=int, default=0)
    b.add_argument('--every', type=int, default=8, help='use every n-th Livox frame of each run as a background')
    b.add_argument('--jobs', type=int, default=None)
    b.add_argument('--scale', type=float, default=1.0, help='scale every scene count (smoke tests)')
    b.add_argument('-o', '--out', required=True)
    t = sub.add_parser('train')
    t.add_argument('--own', required=True)
    t.add_argument('--synth', nargs='+', required=True)
    t.add_argument('--workers', type=int, default=8)
    t.add_argument('--boot', type=int, default=2000)
    t.add_argument('--out-npz', help='save every LORO prediction vector')
    a = ap_.parse_args()
    build(a) if a.cmd == 'build' else train(a)


if __name__ == '__main__':
    main()
