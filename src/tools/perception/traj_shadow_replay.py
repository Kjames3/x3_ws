#!/usr/bin/env python3
"""Replay recorded runs through the trajectory shadow mode, offline.

Reads /pedestrian_states and /odom from each run's bag (orca_cbf_compare's
loader), feeds traj_shadow.TrajShadow exactly what server_x3 --traj-shadow
feeds it live, writes the same NDJSON, and scores it with traj_shadow_score.
Use it to check a model before taking it to the robot.

The deploy model (config/traj_gru_deploy.npz) was trained on every 2026-10
walking run except walk-g-r7, so only that run is out-of-sample.

  source /opt/ros/humble/setup.bash && source install/setup.bash
  python3 src/tools/perception/traj_shadow_replay.py artifacts/cbf-2026-10-16/walk-g-r7 \\
      --out /tmp/shadow-replay
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))            # src/
sys.path.insert(0, str(HERE.parents[1] / "nav"))    # orca_cbf_compare
sys.path.insert(0, str(HERE.parent))

import traj_shadow  # noqa: E402
import traj_shadow_score  # noqa: E402


def replay(run_dir, model, log_path):
    from orca_cbf_compare import load_run, pose_at
    run = load_run(run_dir)
    sh = traj_shadow.TrajShadow(model, log_path)
    t0 = run["odom"][0, 0]
    for t, people in run["ped"]:
        if t < run["odom"][0, 0] or t > run["odom"][-1, 0]:
            continue
        x, y, th = pose_at(run["odom"], t)
        # inverse of publish_pedestrians: /pedestrian_states (px, py) = (z, -x)
        ests = [{"id": i, "z": px, "x": -py, "vx": vx, "vy": vy} for i, px, py, vx, vy in people]
        sh.update(ests, {"x": x, "y": y, "theta": th}, now=t - t0)
    sh.close()
    return sh.stats


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--model", default=str(HERE.parents[3] / "config" / "traj_gru_deploy.npz"))
    ap.add_argument("--out", required=True, help="directory for the per-run NDJSON logs")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    logs = []
    for r in a.runs:
        log = out / f"{Path(r).name}.ndjson"
        log.unlink(missing_ok=True)
        st = replay(r, a.model, log)
        print(f"{Path(r).name}: {st}")
        logs.append(str(log))
    meta, ticks = traj_shadow_score.load(logs)
    traj_shadow_score.report(traj_shadow_score.score(meta, ticks))


if __name__ == "__main__":
    main()
