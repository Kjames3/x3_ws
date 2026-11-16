"""Trajectory shadow mode: odom transform matches training, logging, scoring."""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "src" / "tools" / "perception"))

import traj_shadow as ts  # noqa: E402
import traj_shadow_score as score  # noqa: E402

MODEL = ROOT / "config" / "traj_gru_deploy.npz"


def _estimate(tid, base_xy, base_v):
    """An estimator dict whose /pedestrian_states mapping gives base_xy / base_v."""
    return {"id": tid, "x": -base_xy[1], "z": base_xy[0], "vx": base_v[0], "vy": base_v[1]}


def _walk(shadow, pose_fn, world_fn, vel_world, t_end=4.0, dt=0.1, tid=7):
    rows = []
    for k in range(int(t_end / dt) + 1):
        t = k * dt
        x0, y0, th = pose_fn(t)
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        p_base = R.T @ (np.asarray(world_fn(t)) - (x0, y0))
        v_base = R.T @ np.asarray(vel_world)
        row = shadow.update([_estimate(tid, p_base, v_base)], {"x": x0, "y": y0, "theta": th}, now=t)
        if row:
            rows.append(row)
    return rows


def test_base_from_estimate_matches_publish_pedestrians():
    assert ts.base_from_estimate({"x": 0.4, "z": 2.0, "vx": 0.3, "vy": -0.1}) == (2.0, -0.4, 0.3, -0.1)


def test_odom_positions_are_world_positions_while_the_robot_turns(tmp_path):
    sh = ts.TrajShadow(MODEL, tmp_path / "log.ndjson")
    world = lambda t: (3.0 + 0.8 * t, 1.0)                     # walking +x at 0.8 m/s
    pose = lambda t: (0.5, -0.2, 0.6 * t)                       # robot spinning in place
    rows = _walk(sh, pose, world, (0.8, 0.0))
    last = rows[-1]["tracks"][0]
    t_last = rows[-1]["t"]
    assert abs(last["x"] - world(t_last)[0]) < 1e-3 and abs(last["y"] - 1.0) < 1e-3
    assert abs(last["vx"] - 0.8) < 1e-3 and abs(last["vy"]) < 1e-3
    assert last["used_model"]                                    # > 1 s of history
    # cv sits exactly on the straight line
    for i, h in enumerate(sh.horizons_s):
        assert np.allclose(last["cv"][i], (world(t_last)[0] + 0.8 * h, 1.0), atol=2e-3)
    # the GRU output is exactly what TrajPredictor gives for the same world-frame track:
    # the robot's spin must not leak in. (What the model predicts is not asserted: the
    # 2026-11-08 deploy model damps a steady walk hard, which is what shadow mode measures.)
    from traj_predictor import TrajPredictor
    ref = TrajPredictor(MODEL)
    for k in range(int(4.0 / 0.1) + 1):
        t = k * 0.1
        if t <= t_last + 1e-9:                               # up to the logged tick
            ref.observe(7, t, *world(t), 0.8, 0.0)
    path = ref.predict(7)["path"]
    for i, h in enumerate(sh.horizons_s):
        assert np.allclose(last["gru"][i], path[int(round(h * 10)) - 1], atol=2e-3)
    # cbf_v comes back in BASE axes of the current pose
    th = pose(t_last)[2]
    v_base = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]]) @ (0.8, 0.0)
    assert np.hypot(*(np.asarray(last["cbf_v"]) - v_base)) < 0.25


def test_throttle_logs_at_most_5_hz_but_observes_every_tick(tmp_path):
    sh = ts.TrajShadow(MODEL, tmp_path / "log.ndjson")
    rows = _walk(sh, lambda t: (0, 0, 0), lambda t: (2.0, 0.5 * t), (0.0, 0.5), t_end=2.0, dt=0.05)
    assert 8 <= len(rows) <= 11                                 # 2 s at <= 5 Hz (float steps)
    lines = (tmp_path / "log.ndjson").read_text().splitlines()
    assert json.loads(lines[0])["type"] == "meta" and len(lines) == len(rows) + 1


def test_stale_odom_skips_and_is_counted():
    sh = ts.TrajShadow(MODEL)
    assert sh.update([_estimate(1, (2, 0), (0, 0))], {"x": 0, "y": 0, "theta": 0},
                     now=0.0, odom_age_s=2.0) is None
    assert sh.stats["skipped_stale_odom"] == 1 and sh.stats["predictions"] == 0


def test_non_people_are_ignored():
    sh = ts.TrajShadow(MODEL)
    est = dict(_estimate(1, (2, 0), (0, 0)), is_person=False)
    row = sh.update([est], {"x": 0, "y": 0, "theta": 0}, now=0.0)
    assert row["tracks"] == []


def test_scorer_grades_against_the_track_itself(tmp_path):
    log = tmp_path / "log.ndjson"
    sh = ts.TrajShadow(MODEL, log)
    _walk(sh, lambda t: (0.0, 0.0, 0.2), lambda t: (1.0 + 0.6 * t, -0.5), (0.6, 0.0), t_end=8.0)
    sh.close()
    meta, ticks = score.load([str(log)])
    out = score.score(meta, ticks)
    rows = {r["h"]: r for r in out["subsets"]["moving"]}
    assert rows[1.0]["n"] > 10
    assert rows[1.0]["cv_mean"] < 0.01                          # straight line: cv is exact
    assert rows[3.0]["n"] < rows[0.5]["n"]                       # fewer windows have 3 s of future
