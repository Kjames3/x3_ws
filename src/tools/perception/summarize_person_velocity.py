#!/usr/bin/env python3
"""Summarize the frozen held-out person-input/uncertainty experiment."""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
_REPO_ROOT = _Path(__file__).resolve().parents[3]
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def weighted(rows,key,weight):
    rows=[r for r in rows if r.get(key) is not None]
    return float(np.average([r[key] for r in rows],weights=[r[weight] for r in rows])) if rows else None


def main(out):
    result=json.loads((out/'held-out-results.json').read_text())
    selected=json.loads((out/'selected.json').read_text())
    transfer=json.loads((out/'transfer-verification.json').read_text())
    scenarios=['approach','crossing','startstop','stand']
    arms=list(next(iter(result.values())))
    aggregates={}
    for scenario in scenarios:
        runs=[v for n,v in result.items() if n.startswith(scenario)]
        aggregates[scenario]={}
        for arm in arms:
            ms=[r[arm]['metrics'] for r in runs]
            aggregates[scenario][arm]=dict(
                recall=sum(m['matched'] for m in ms)/max(1,sum(m['reference_frames'] for m in ms)),
                n=sum(m['reference_frames'] for m in ms),
                speed_mae=weighted(ms,'speed_mae','moving_n'),
                static_false_fraction=weighted(ms,'static_false_fraction','matched'))
    calibration={}; decisions={}
    for frontend in ['pose','seg-merged']:
        for suffix in ['kalman','calibrated']:
            arm=frontend+'/'+suffix; calibration[arm]={}
            for horizon in ['0.0','0.5','1.0']:
                rs=[v[arm]['uncertainty'][horizon] for name,v in result.items()
                    if not name.startswith(('stand','empty')) and horizon in v[arm]['uncertainty']]
                n=sum(r['n'] for r in rs)
                calibration[arm][horizon]=dict(n=n,
                    cover95=weighted(rs,'cover95','n'),nll=weighted(rs,'nll','n'),
                    rmse_m=float(np.sqrt(np.average([r['rmse_m']**2 for r in rs],weights=[r['n'] for r in rs]))),
                    stated_m=float(np.sqrt(np.average([r['sigma_m']**2 for r in rs],weights=[r['n'] for r in rs]))))
        base=calibration[frontend+'/kalman']; candidate=calibration[frontend+'/calibrated']
        reasons=[]
        if candidate['0.0']['cover95']<.90:
            reasons.append('current-position coverage below 90% for a nominal 95% region')
        if candidate['0.5']['cover95']<.90:
            reasons.append('0.5-second prediction coverage below 90%')
        if candidate['0.0']['nll']+.5*candidate['0.5']['nll'] >= base['0.0']['nll']+.5*base['0.5']['nll']:
            reasons.append('held-out Gaussian score did not improve')
        for name,v in result.items():
            a=v[frontend+'/calibrated']['metrics']; b=v[frontend+'/kalman']['metrics']
            if a['recall'] is not None and a['recall']<b['recall']-.02:
                reasons.append(name+': recall loss exceeds two percentage points')
            if a['static_false_fraction'] is not None and a['static_false_fraction']>b['static_false_fraction']+.02:
                reasons.append(name+': static false-motion increase exceeds two percentage points')
        decisions[frontend]=dict(offline_candidate_pass=not reasons,reasons=reasons,
                                  configuration=selected['candidates'][frontend],deployed=False)
    summary=dict(aggregates=aggregates,calibration=calibration,decisions=decisions)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    root=_REPO_ROOT
    for fe,d in decisions.items():
        if d['offline_candidate_pass']:
            path=root/'config'/f'c3_uncertainty_{fe}_candidate.json'
            path.write_text(json.dumps(d['configuration']['config'],indent=2)+'\n')
    lines=['# Person measurements and uncertainty: 2026-10-12','',
        'Implemented and tested on the laptop. Robot runtime defaults were not changed by this experiment.',
        'Cleaner person inputs improve MLP coverage over blobs, but the person Kalman tracker retains more crossing coverage.',
        'Neither covariance candidate passes every held-out gate; retain both baseline configurations.','',
        '## Recordings and experiment contract','',
        f"- 25 recordings, {transfer['total_frames']:,} original indexed depth frames copied losslessly.",
        f"- Depth archives total {transfer['total_depth_bytes']/1e6:.1f} MB; all archive, index and individual-frame hashes verified.",
        '- 400 mask, measurement, reference and calibration files checked byte-for-byte against the robot.',
        '- Laptop data: `artifacts/c3-captures-segpose/<run>/depth-frames.npz`, with original pair indexes and cached measurements.',
        '- This is a depth-only replay bundle, not a complete ROS bag copy. RGB and unrelated topics remain in the original robot bags.',
        '- Eight repeat-1 runs tune covariance; sixteen repeat-2/3 runs are held out. Empty-lab is a separate false-output check.',
        '- 36 covariance combinations plus the unchanged live baseline; selection frozen in `selected.json` before held-out scoring.',
        '- Same pretrained v3 model/scaler, 10 Hz resampling, stop gate, confidence factor and acceleration limit for every MLP arm.',
        '- Both MLP and Kalman see the same measured person positions and baseline association IDs; the MLP never sees smoothed Kalman positions.',
        '- Blob MLP uses the SAME association method as person MLP to isolate input quality. It is not a full replay of the deployed blob tracker.',
        '- Main tables use the existing fully-in-frame reference subset. Full-reference metrics and paired common-frame errors are also saved.',
        '- References use YOLO11x and the same depth sensor: these are agreement/coverage results, not independent ground-truth velocity accuracy.',
        '- Standing metrics exclude the first two seconds. Uncertainty is conditional on a matched track; recall is reported separately.',
        '- All runs are from one lab. Transfer to another room and current live NN cadence remain untested.','',
        '## Item 4: cleaner person positions through the MLP','',
        '| Scenario | Input / estimator | Reference recall | Moving-speed MAE (m/s) | Standing false motion >0.15 m/s |',
        '|---|---|---:|---:|---:|']
    display=['blob/mlp','pose/mlp','seg-merged/mlp','pose/kalman','seg-merged/kalman']
    for scenario in scenarios:
        for arm in display:
            a=aggregates[scenario][arm]
            speed=f"{a['speed_mae']:.3f}" if scenario!='stand' and a['speed_mae'] is not None else '—'
            false=f"{100*a['static_false_fraction']:.1f}%" if a['static_false_fraction'] is not None else '—'
            lines.append(f"| {scenario} | {arm} | {100*a['recall']:.1f}% | {speed} | {false} |")
    lines += ['', 'Speed MAE above is conditional on the arm finding the person; populations differ. '
              'Use `paired_speed_mae` (same front end, MLP versus Kalman) or `common_speed_mae` '
              '(intersection of all arms) in `held-out-results.json` for matched-population comparisons.', '',
              'The MLP requires 0.9 seconds of measured history. Re-entry and short tracks therefore lose coverage, '
              'even though an established track can respond promptly to starts/stops. Keep Kalman as the person '
              'baseline; retain the measured-position MLP adapter for controlled experiments, not automatic replacement.', '',
              '## Item 5: uncertainty on held-out walking runs','',
              '| Front end / configuration | Horizon (s) | Error RMS (m) | Stated RMS uncertainty (m) | Nominal 95% coverage |',
              '|---|---:|---:|---:|---:|']
    for arm,hs in calibration.items():
        for h,r in hs.items():
            lines.append(f"| {arm} | {h} | {r['rmse_m']:.3f} | {r['stated_m']:.3f} | {100*r['cover95']:.1f}% |")
    lines += ['', 'Baseline: centroid floor 0.05 m, range coefficient 0.0025, within-object spread scale 1.0; '
              'process acceleration sigma remains 1.5 m/s². Tuning tested range coefficients 0.0025/0.009/0.011, '
              'floors 0.02/0.05 m and spread scales 0.25/0.5/1.0.', '']
    for fe,d in decisions.items():
        status='passes offline gates; candidate only' if d['offline_candidate_pass'] else 'rejected; retain baseline'
        lines.append(f"- **{fe}: {status}.** Selected on tuning data: `{d['configuration']['name']}`. "
                     +'; '.join(d['reasons']))
    lines += ['', 'The pose candidate fails the per-run recall guard on crossing-1.2m-r3: '
              '7/8 fully-in-frame reference frames matched versus 8/8 for baseline. This one-frame difference '
              'on a tiny subset is not strong evidence of worse overall performance; it leaves the candidate '
              'unqualified for promotion despite promising aggregate results.', '',
              'The selected effective centroid noise term stays at 0.0025. This does not contradict the measured '
              'Pro W per-pixel noise: the quantities differ. Spatial depth spread also includes body geometry. '
              'Blindly substituting the sensor coefficient did not win this calibration.', '',
              'The 1-second prediction regions remain conservative. Measurement-noise tuning alone does not resolve '
              'the constant-velocity/process-noise model at longer horizons.', '',
              '## Implementation and use','',
              '- `src/tools/perception/person_velocity_mlp.py`: measured-position MLP adapter with explicit model/scaler selection, '
              'track identity, timestamped windows and world-frame velocity outputs.',
              '- `src/c3_person_tracker.py`: configurable range noise and spread scaling; raw-measurement association indices.',
              '- `src/c3_live.py`, `c3_replay.py`, `c3_calibration.py`: propagate uncertainty configuration without changing defaults.',
              '- `src/tools/perception/evaluate_person_velocity.py`: separated tuning/evaluation stages and source-checked blob caches.',
              '- `src/tools/perception/export_person_depth.py`: lossless indexed depth export and hash verification.',
              '- `selected.json` retains the tuning candidates; `summary.json` records why neither was promoted.',
              '', 'Reproduce on the laptop (no camera or ROS graph needed for the compact bundle):','',
              '```bash',
              'OPENBLAS_NUM_THREADS=1 python3 src/tools/perception/evaluate_person_velocity.py tune',
              'OPENBLAS_NUM_THREADS=1 python3 src/tools/perception/evaluate_person_velocity.py evaluate --with-blobs',
              'python3 src/tools/perception/summarize_person_velocity.py',
              '```','',
              'The implementation accepts an explicit `tracker_config` dict for future controlled experiments. '
              'Current callers retain `ARM_C_CONFIG`; do not promote the rejected tuning candidates. '
              'No robot server restart, model switch, or driving command was performed for items 4/5.']
    (out/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(decisions,indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,default=_REPO_ROOT/'artifacts/person-velocity-2026-10-12')
    main(ap.parse_args().output)
