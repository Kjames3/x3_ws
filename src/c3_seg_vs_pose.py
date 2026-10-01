#!/usr/bin/env python3
"""Seg vs pose as the C3 person front end, on identical recordings.

  python3 src/c3_seg_vs_pose.py RUN_DIR... --output OUT

For every run: cache yolo26n-pose boxes and yolo26n-seg boxes + silhouettes
(c3_replay detect), build the yolo11x reference (c3_score prepare), then score
arm C twice -- pose boxes with torso depth, seg silhouettes with silhouette
depth -- with the unchanged c3_score metrics (recall, false motion, speed,
stop/go delay). Arms A/B do not depend on the front end and are identical in
both reports.

Adds a taped-distance check the reference cannot give (it shares the depth
sensor): runs named stand-<d>m-rN / stand-side-<d>m-rN have the person's toes
on a mark <d> m from the camera lens, so arm C's forward distance has a known
target. Torso depth sits ~0.1-0.15 m behind the toes for both front ends, so
compare bias between front ends, not against zero.
Offline only; nothing reaches the robot or the CBF.
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_score as cs  # noqa: E402
from person_box_merge import MERGE_CONTAIN, group_overlapping  # noqa: E402

FRONT_ENDS = {
    'pose': dict(model='models/yolo26n-pose.pt', name='c3-det-pose.json', masks=False),
    'seg': dict(model='models/yolo26n-seg.pt', name='c3-det-seg.json', masks=True),
    # Post-hoc (added 2026-10-01 after the first scoring): seg split close
    # people into overlapping partial boxes (e.g. left half, right half,
    # whole), each becoming its own track. Same seg detections, merged.
    'seg-merged': dict(derived_from='seg', name='c3-det-seg-merged.json'),
}

# The live OAK's nn_conf. The replay default (0.35) admits the lab's
# towel-on-chair phantom for both models, which is not what would run live.
CONF = 0.5
STAND_RE = re.compile(r'^stand-(side-)?(\d+(?:\.\d+)?)m-r\d+$')


def stand_accuracy(run_out, run_name):
    """Forward distance of arm C's measured, confirmed outputs vs the tape."""
    m = STAND_RE.match(run_name)
    path = run_out / 'armC_tracks.csv'
    if not m or not path.exists():
        return None
    target = float(m.group(2))
    with path.open() as f:
        rows = [r for r in csv.DictReader(f)
                if r['confirmed'] == 'True' and r['measured'] == 'True']
    if not rows:
        return dict(target_m=target, side_on=bool(m.group(1)), n=0)
    # The person is the track with the most measured outputs; other ids are
    # phantoms or re-acquisitions, counted but kept out of bias/jitter.
    ids = [r['id'] for r in rows]
    person = max(set(ids), key=ids.count)
    z = np.array([float(r['z']) for r in rows if r['id'] == person])
    return dict(target_m=target, side_on=bool(m.group(1)), n=int(z.size),
                bias_m=float(np.median(z) - target),
                jitter_m=float(np.percentile(z, 84) - np.percentile(z, 16)) / 2,
                track_ids=len(set(ids)))


def merge_boxes(boxes, threshold=MERGE_CONTAIN):
    """Index groups of detection dicts ({'xyxy': ...}) that are one person."""
    return group_overlapping([b['xyxy'] for b in boxes], threshold)


def write_merged(run, src_name, dst_name):
    """One box per merged group: union box, max confidence, OR of silhouettes."""
    src = json.loads((run / src_name).read_text())
    masks = np.load(run / src['masks_file'])
    packed, n_in, n_out = {}, 0, 0
    for fr in src['frames']:
        merged = []
        for k, group in enumerate(merge_boxes(fr['boxes'])):
            bs = [fr['boxes'][i] for i in group]
            box = dict(xyxy=[min(b['xyxy'][0] for b in bs), min(b['xyxy'][1] for b in bs),
                             max(b['xyxy'][2] for b in bs), max(b['xyxy'][3] for b in bs)],
                       conf=max(b['conf'] for b in bs), merged_from=len(bs))
            sils = [np.unpackbits(masks[b['mask']]) for b in bs if 'mask' in b]
            if sils:
                key = f"{fr['index']}_{k}"
                packed[key] = np.packbits(np.bitwise_or.reduce(sils).astype(bool))
                box['mask'] = key
            merged.append(box)
        n_in += len(fr['boxes']); n_out += len(merged)
        fr['boxes'] = merged
    mpath = rp.masks_path(run, dst_name)
    np.savez_compressed(mpath, **packed)
    src.update(masks_file=mpath.name, merged=dict(from_file=src_name, contain=MERGE_CONTAIN))
    (run / dst_name).write_text(json.dumps(src) + '\n')
    print(f'{run.name}: merged {n_in} seg boxes -> {n_out}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--front-ends', default=','.join(FRONT_ENDS),
                    help='comma list to (re)score; others are read back from a '
                         'previous seg_vs_pose.json in --output (default: all)')
    ap.add_argument('--tracker', choices=['default', 'live'], default='default',
                    help="arm C tracker settings: c3_person_tracker.DEFAULT_CONFIG or "
                         "the robot's c3_live.ARM_C_CONFIG. Use a separate --output per choice.")
    args = ap.parse_args()
    runs = sorted(r for r in args.runs if (r / 'pair-index.json').exists())
    todo = args.front_ends.split(',')
    config_c = None
    if args.tracker == 'live':
        from c3_live import ARM_C_CONFIG
        config_c = dict(ARM_C_CONFIG)

    for run in runs:
        for label, fe in FRONT_ENDS.items():
            if 'derived_from' in fe or (run / fe['name']).exists():
                continue
            print(f'{run.name}: detect {fe["model"]}', flush=True)
            rp.detect(run, fe['model'], CONF, fe['name'], fe['masks'])
        if 'seg-merged' in todo and not (run / FRONT_ENDS['seg-merged']['name']).exists():
            write_merged(run, FRONT_ENDS['seg']['name'], FRONT_ENDS['seg-merged']['name'])
        cs.prepare(run)

    prev = args.output / 'seg_vs_pose.json'
    result = json.loads(prev.read_text()) if prev.exists() else {}
    for label in todo:
        out = args.output / label
        report = cs.score(runs, out, config_c=config_c, detections_name=FRONT_ENDS[label]['name'])
        result[label] = dict(
            aggregate_C=cs.aggregate(report)['C'],
            stand={r.name: stand_accuracy(out / r.name, r.name) for r in runs
                   if STAND_RE.match(r.name)})
    args.output.mkdir(parents=True, exist_ok=True)
    prev.write_text(json.dumps(result, indent=1) + '\n')

    # Side-by-side table of every arm C metric each front end produced.
    labels = [fe for fe in FRONT_ENDS if fe in result]
    keys = sorted(set().union(*(result[fe]['aggregate_C'] for fe in labels)))
    lines = ['| metric | ' + ' | '.join(labels) + ' |', '|---' * (len(labels) + 1) + '|']
    for k in keys:
        cells = []
        for label in labels:
            v = result[label]['aggregate_C'].get(k)
            cells.append('--' if v is None else ', '.join(
                f'{m}={x:.3g}' if isinstance(x, float) else f'{m}={x}' for m, x in v.items()))
        lines.append(f'| {k} | ' + ' | '.join(cells) + ' |')
    lines += ['', '| stand run | ' + ' | '.join(f'{fe} bias / jitter / ids' for fe in labels) + ' |',
              '|---' * (len(labels) + 1) + '|']
    for name in sorted(result[labels[0]]['stand']):
        cells = []
        for label in labels:
            s = result[label]['stand'].get(name)
            cells.append('no output' if not s or not s['n'] else
                         f"{s['bias_m']:+.3f} m / {s['jitter_m']:.3f} m / {s['track_ids']}")
        lines.append(f'| {name} | ' + ' | '.join(cells) + ' |')
    (args.output / 'RESULTS.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
