#!/usr/bin/env python3
"""Score saved lidar person classifiers per run on a lidar_person_dataset.py file.

    python3 src/tools/perception/lidar_person_score_runs.py runs.npz model.joblib [model2.joblib ...]

Models come from lidar_person_classifier.py --save.  Typical use: build an npz from runs
the models never saw (e.g. the apartment seated / close / occluded runs of 2026-11-07,
with seat positions in label_overrides.json) and check each model on them.

Per run, at threshold 0.5:
  seated recall   candidates labelled 'sitting' (label 2, hand overrides) scored person
  person recall   label-1 candidates scored person, also for those closer than CLOSE_M
  neg FP          label-0 candidates (camera-checked + persistent) scored person
  AP              person+sitting vs not-person, threshold-free

These score the classifier on candidates that EXIST.  A person the extractor never
produced a candidate for does not appear here; compare the candidate counts printed by
lidar_person_dataset.py against the run's frame count for that.
"""
import argparse

import joblib
import numpy as np
from sklearn.metrics import average_precision_score

CLOSE_M = 0.8
THRESH = 0.5


def score(d, model_path):
    names = list(d['feature_names'])
    m = joblib.load(model_path)
    p = m['model'].predict_proba(d['X'][:, [names.index(f) for f in m['features']]])[:, 1]
    print(f'## {model_path}')
    for run in sorted(set(d['run'])):
        k = d['run'] == run
        y, s, rng = d['y'][k], p[k], d['X'][k, 0]
        pos, neg = (y == 1) | (y == 2), y == 0
        line = f'{run:22s}'
        if (y == 2).any():
            line += f' seated recall {np.mean(s[y == 2] > THRESH):.2f} (n {np.sum(y == 2)})'
        if (y == 1).any():
            close = (y == 1) & (rng < CLOSE_M)
            c = f'{np.mean(s[close] > THRESH):.2f}' if close.any() else '--'
            line += f' person recall {np.mean(s[y == 1] > THRESH):.2f} (n {np.sum(y == 1)}; <{CLOSE_M} m {c})'
        if neg.any():
            line += f' | neg FP {np.mean(s[neg] > THRESH):.3f} (n {neg.sum()})'
        if pos.any() and neg.any():
            line += f' | AP {average_precision_score(pos[pos | neg], s[pos | neg]):.3f}'
        print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('npz')
    ap.add_argument('models', nargs='+')
    a = ap.parse_args()
    d = np.load(a.npz, allow_pickle=True)
    for mp in a.models:
        score(d, mp)


if __name__ == '__main__':
    main()
