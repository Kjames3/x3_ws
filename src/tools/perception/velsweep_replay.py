#!/usr/bin/env python3
"""Score candidate velocity MLPs as arm A of the C3 replay on recorded lab runs.

c3_replay's arm A is the deployed VelocityEstimator._step with the v3 weights.
This swaps only the weights (and their paired scaler), replays every run, and
scores against the YOLO11x reference with c3_score, so the model is the only
variable between candidates. Arms B and C are replayed too but do not change.

    source /opt/ros/humble/setup.bash
    python3 src/tools/perception/velsweep_replay.py --models v3,artifacts/velsweep4-2026-10-02/velocity_mlp_prow_j20_deploy.torchscript \
        --output artifacts/velsweep4-2026-10-02/replay artifacts/c3-captures/*-r[23]

A model path must be named velocity_mlp_<name>.torchscript with
scaler_params_<name>.json beside it (VelocityEstimator._scaler_path_for).
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
_REPO_ROOT = _Path(__file__).resolve().parents[3]
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import json
import sys
from pathlib import Path

ROOT = _REPO_ROOT
sys.path.insert(0, str(ROOT / 'src'))

import c3_replay as rp  # noqa: E402
import c3_score as cs  # noqa: E402

V3 = ROOT / 'src' / 'velocity_mlp_v3.torchscript'


def use_model(path):
    def make(directory):
        import torch
        import velocity_estimator as ve
        torch.set_num_threads(1)
        snap = directory / 'snapshot' / 'config' / 'camera_ground_plane.json'
        if snap.exists():
            ve.GROUND_PLANE_CONFIG_PATH = str(snap)
        est = ve.VelocityEstimator(None, None, model_path=str(path))
        if est._model is None:
            raise RuntimeError('%s failed to load' % path)
        return est
    rp.make_v3 = make


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', type=Path, nargs='+')
    ap.add_argument('--models', required=True, help='comma-separated: v3 or model paths')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()

    table = {}
    for spec in args.models.split(','):
        path = V3 if spec == 'v3' else Path(spec).resolve()
        name = 'v3' if spec == 'v3' else path.stem[len('velocity_mlp_'):]
        print('===== %s' % name, flush=True)
        use_model(path)
        report = cs.score(args.runs, args.output / name)
        table[name] = cs.aggregate(report)['A']
        (args.output / 'armA_by_model.json').write_text(json.dumps(table, indent=1) + '\n')
    print(json.dumps(table, indent=1))


if __name__ == '__main__':
    main()
