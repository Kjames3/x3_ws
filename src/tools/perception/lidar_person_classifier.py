#!/usr/bin/env python3
"""Train and score a person / not-person classifier on lidar cluster features.

    python3 src/tools/perception/lidar_person_classifier.py --own own.npz [--sit sit.npz]
            [--save model.joblib]

Inputs come from lidar_person_dataset.py (the robot's bags, auto-labelled) and
sit_person_dataset.py (SiT boxes).  Model: sklearn HistGradientBoostingClassifier on
cluster_features.NAMES (no reflectivity: it does not transfer between sensors).

Seated people (label 2).  On the robot (hand overrides) they count as people (user
decision 2026-11-01: someone sitting can stand up into the robot's path).  SiT's
'Pedestrain_sitting' boxes are LEFT OUT by default: counted as people they took SiT ->
robot AP from 0.81 to 0.39 (a seated pedestrian is usually one cluster with the bench or
table, which teaches 'furniture = person').  --sit-sitting-as-person brings them back;
--exclude-sitting leaves the robot's seated people out as well.

Experiments, all scored on held-out data, split by RUN (never frames of the same run on
both sides):
  own        leave-one-run-out on the robot's bags
  sit->own   trained on SiT only, scored on every robot run
  own+sit    leave-one-run-out, SiT added to every training fold
  own->sit   trained on all robot runs, scored on SiT

Scores per range band: average precision (threshold-free) and precision / recall at
0.5.  Robot negatives come in two strengths ('static_neg': the camera looked and saw
nobody; 'persistent': fixed structure anywhere) and are reported against the person
labels both together and with the camera-checked ones only.  The track rows average
the score over each tracklet first, which is roughly what a tracker that accumulates
evidence would see.
"""
import argparse

import numpy as np
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.metrics import average_precision_score

BANDS = ((0, 2), (2, 4), (4, 6), (6, 8), (8, 12))


def _sitting(d, exclude):
    """Label 2 (seated person) -> 1, or leave it out (-1) with exclude."""
    y = d['y'].copy()
    y[y == 2] = -1 if exclude else 1
    d['y'] = y
    return d


def model():
    return HistGradientBoostingClassifier(max_iter=300, learning_rate=0.08, max_leaf_nodes=31,
                                          l2_regularization=1.0, class_weight='balanced',
                                          random_state=0)


def _usable(d, weak=True):
    y = d['y']
    m = (y == 0) | (y == 1)
    if not weak and 'source' in d:
        m &= d['source'] != 'persistent'
    return m


def _line(name, y, p, rng):
    cells = []
    for lo, hi in BANDS:
        m = (rng >= lo) & (rng < hi)
        npos, nneg = int((y[m] == 1).sum()), int((y[m] == 0).sum())
        if npos < 10 or nneg < 10:
            cells.append(f'{"--":>17s}')
            continue
        ap = average_precision_score(y[m], p[m])
        pr = ((p[m] >= 0.5) & (y[m] == 1)).sum() / max(1, (p[m] >= 0.5).sum())
        rc = ((p[m] >= 0.5) & (y[m] == 1)).sum() / max(1, npos)
        cells.append(f'{ap:.2f} {pr:.2f}/{rc:.2f} {npos // 100:>2d}h')
    ap_all = average_precision_score(y, p) if 0 < y.sum() < len(y) else float('nan')
    print(f'  {name:26s} AP {ap_all:.3f} |' + ' |'.join(cells))


def header():
    print(f'  {"":26s}          |' + ' |'.join(f'{f"{lo}-{hi} m AP P/R":>17s}' for lo, hi in BANDS))


def report(title, d, p, mask):
    """d: dataset dict, p: scores aligned with d, mask: rows that were scored."""
    print(title)
    header()
    y, rng = d['y'], d['X'][:, 0]
    src = d['source']
    m = mask & ((y == 0) | (y == 1))
    _line('all labels', y[m], p[m], rng[m])
    if 'track' in d:
        m2 = m & (src != 'persistent')
        _line('camera-checked negs only', y[m2], p[m2], rng[m2])
        # tracklet average
        tr = d['track'][m]
        uniq, inv = np.unique(tr, return_inverse=True)
        pm = np.bincount(inv, weights=p[m]) / np.bincount(inv)
        _line('tracklet-averaged', y[m], pm[inv], rng[m])


def own_loro(own, extra=None, feats=None):
    runs = np.unique(own['run'])
    p = np.full(len(own['y']), np.nan)
    use = _usable(own)
    for r in runs:
        tr = use & (own['run'] != r)
        Xtr, ytr = own['X'][tr][:, feats], own['y'][tr]
        if extra is not None:
            eu = _usable(extra)
            Xtr = np.vstack([Xtr, extra['X'][eu][:, feats]])
            ytr = np.r_[ytr, extra['y'][eu]]
        if len(np.unique(ytr)) < 2:
            continue
        te = own['run'] == r
        p[te] = model().fit(Xtr, ytr).predict_proba(own['X'][te][:, feats])[:, 1]
    return p, ~np.isnan(p)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--own', required=True)
    ap.add_argument('--sit')
    ap.add_argument('--save', help='fit on all robot data (+SiT with --save-with-sit) and save')
    ap.add_argument('--save-with-sit', action='store_true')
    ap.add_argument('--exclude-sitting', action='store_true',
                    help="leave the robot's seated people out instead of counting them as people")
    ap.add_argument('--sit-sitting-as-person', action='store_true',
                    help="count SiT's seated pedestrians as people (default: left out)")
    ap.add_argument('--drop', default='', help='comma-separated feature names to leave out')
    a = ap.parse_args()
    own = _sitting(dict(np.load(a.own, allow_pickle=True)), a.exclude_sitting)
    names = [str(n) for n in own['feature_names']]
    feats = [i for i, n in enumerate(names) if n not in set(filter(None, a.drop.split(',')))]
    print(f'features: {len(feats)} of {len(names)}')
    u = _usable(own)
    print(f'robot: {int((own["y"] == 1).sum())} person, {int((own["source"] == "static_neg").sum())} camera-checked'
          f' + {int((own["source"] == "persistent").sum())} persistent not-person, runs {len(np.unique(own["run"]))}'
          f'  (cells: AP precision/recall@0.5, persons in hundreds)')

    p, m = own_loro(own, feats=feats)
    report('\n[own] leave-one-run-out, robot bags only', own, p, m)

    sit = None
    if a.sit:
        sit = dict(np.load(a.sit, allow_pickle=True))
        n_sit = int((sit['y'] == 2).sum())
        sit = _sitting(sit, not a.sit_sitting_as_person)
        su = _usable(sit)
        print(f'\nSiT: {int((sit["y"] == 1).sum())} person, {int((sit["y"] == 0).sum())} not-person '
              f'({n_sit} sitting {"counted as person" if a.sit_sitting_as_person else "left out"})')
        clf = model().fit(sit['X'][su][:, feats], sit['y'][su])
        report('\n[sit->own] trained on SiT, scored on robot bags', own, clf.predict_proba(own['X'][:, feats])[:, 1], u)
        p2, m2 = own_loro(own, extra=sit, feats=feats)
        report('\n[own+sit] leave-one-run-out with SiT in every fold', own, p2, m2)
        clf = model().fit(own['X'][u][:, feats], own['y'][u])
        report('\n[own->sit] trained on robot bags, scored on SiT', sit, clf.predict_proba(sit['X'][:, feats])[:, 1], su)
        # within SiT, leave-one-clip-out, for scale
        ps = np.full(len(sit['y']), np.nan)
        for r in np.unique(sit['run']):
            tr = su & (sit['run'] != r)
            ps[sit['run'] == r] = model().fit(sit['X'][tr][:, feats], sit['y'][tr]).predict_proba(
                sit['X'][sit['run'] == r][:, feats])[:, 1]
        report('\n[sit] leave-one-clip-out within SiT (for scale)', sit, ps, su)

    if a.save:
        import joblib
        X, y = own['X'][u][:, feats], own['y'][u]
        if a.save_with_sit and sit is not None:
            su = _usable(sit)
            X, y = np.vstack([X, sit['X'][su][:, feats]]), np.r_[y, sit['y'][su]]
        joblib.dump({'model': model().fit(X, y), 'features': [names[i] for i in feats]}, a.save)
        print(f'saved {a.save}')


if __name__ == '__main__':
    main()
