"""person_paths: GUI path ribbons for people (display only, X3_PERSON_PATH)."""
import math
from pathlib import Path

import numpy as np
import pytest

import person_paths as pp

MODEL = Path(__file__).resolve().parents[1] / "config" / "traj_gru_deploy.npz"


def walker(t0, n, dt, speed, w=0.0, x0=0.0, y0=0.0, th0=0.0, tid="L1"):
    """Rows of a person walking at `speed` with turn rate w (odom), as the lidar tracker sends them."""
    rows, x, y, th = [], x0, y0, th0
    for i in range(n):
        rows.append({"id": tid, "stamp": t0 + i * dt, "mode": "lidar", "odom_x": x, "odom_y": y,
                     "odom_vx": speed * math.cos(th), "odom_vy": speed * math.sin(th)})
        x += speed * math.cos(th) * dt
        y += speed * math.sin(th) * dt
        th += w * dt
    return rows


def feed(paths, rows, pose, mono0=100.0, dt=0.1):
    out = None
    for i, r in enumerate(rows):
        out = paths.annotate([r], "lidar", pose, now=mono0 + i * dt, stamp_key="stamp")
    return out[0]


POSE0 = {"x": 0.0, "y": 0.0, "theta": 0.0}


def test_off_leaves_payload_untouched():
    assert pp.from_env("unused", env="off") is None
    assert pp.from_env("unused", env="") is None
    assert pp.from_env("unused", env="bogus") is None
    c3, lp, ve = [{"id": 1}], [{"id": "L1"}], [{"id": 2}]
    out = pp.annotate_telemetry(None, POSE0, c3, lp, ve)
    assert out[0] is c3 and out[1] is lp and out[2] is ve


def test_straight_walker_gets_a_straight_16_point_path():
    paths = pp.PersonPaths("ctrv")
    r = feed(paths, walker(1.79e9, 15, 0.1, 1.0), POSE0)
    p = np.array(r["path_rel"])
    assert p.shape == (pp.N_POINTS, 2)
    assert p[0].tolist() == [0.0, 0.0]
    assert p[-1] == pytest.approx([2.0, 0.0], abs=0.02)
    assert len(r["path_sigma"]) == pp.N_POINTS and r["path_sigma"][-1] > r["path_sigma"][0]
    assert r["path_alts"] == []


def test_input_dicts_are_not_modified():
    paths = pp.PersonPaths("cv")
    rows = walker(1.79e9, 15, 0.1, 1.0)
    feed(paths, rows, POSE0)
    assert all("path_rel" not in r for r in rows)


def test_sustained_left_turn_bends_left_and_cv_does_not():
    rows = walker(1.79e9, 20, 0.1, 1.0, w=0.8)
    curved = np.array(feed(pp.PersonPaths("ctrv"), rows, POSE0)["path_rel"])
    straight = np.array(feed(pp.PersonPaths("cv"), rows, POSE0)["path_rel"])
    # heading at the last sample, in base axes (robot at the origin, theta 0)
    th = 0.8 * 0.1 * 19
    u = np.array([math.cos(th), math.sin(th)])
    lateral = lambda p: u[0] * p[-1, 1] - u[1] * p[-1, 0]      # + = left of the walking line
    assert lateral(curved) > 0.1
    assert abs(lateral(straight)) < 0.01          # (3-decimal rounding)
    # damped: the 2 s bend stays under w * tau (no spiral)
    end_heading = math.atan2(*(curved[-1] - curved[-2])[::-1]) - th
    assert 0 < end_heading <= pp.CTRV["max_w"] * pp.CTRV["tau_s"] + 1e-6


def test_offsets_are_in_base_axes_with_the_robot_yawed():
    # robot at (1, 2) facing odom +y; the person walks along odom +y = robot forward
    pose = {"x": 1.0, "y": 2.0, "theta": math.pi / 2}
    rows = walker(1.79e9, 15, 0.1, 1.0, x0=1.0, y0=4.0, th0=math.pi / 2)
    p = np.array(feed(pp.PersonPaths("cv"), rows, pose)["path_rel"])
    assert p[-1] == pytest.approx([2.0, 0.0], abs=0.02)        # forward, not left
    # facing odom +y, the robot's left is odom -x
    rows = walker(1.79e9, 15, 0.1, 1.0, x0=1.0, y0=4.0, th0=math.pi)
    p = np.array(feed(pp.PersonPaths("cv"), rows, pose)["path_rel"])
    assert p[-1] == pytest.approx([0.0, 2.0], abs=0.02)        # all left, none forward


def test_slow_person_gets_no_path():
    r = feed(pp.PersonPaths("ctrv"), walker(1.79e9, 15, 0.1, 0.05), POSE0)
    assert "path_rel" not in r


def test_coasting_rows_are_drawn_but_not_learned():
    paths = pp.PersonPaths("ctrv")
    rows = walker(1.79e9, 15, 0.1, 1.0)
    feed(paths, rows, POSE0)
    n = len(paths._obs[("lidar", "L1")])
    coast = dict(rows[-1], mode="coasting", stamp=rows[-1]["stamp"] + 0.1, odom_x=9.0)
    r = paths.annotate([coast], "lidar", POSE0, now=200.0, stamp_key="stamp")[0]
    assert len(paths._obs[("lidar", "L1")]) == n
    assert "path_rel" in r


def test_stale_tracks_forgotten_on_the_annotate_clock():
    # stamps are wall clock (1.79e9), annotate's clock is monotonic: must not mix them
    paths = pp.PersonPaths("cv")
    feed(paths, walker(1.79e9, 15, 0.1, 1.0), POSE0, mono0=100.0)
    assert ("lidar", "L1") in paths._obs
    paths.annotate([], "lidar", POSE0, now=101.5 + 0.5)
    assert ("lidar", "L1") in paths._obs
    paths.annotate([], "lidar", POSE0, now=101.5 + pp.STALE_S + 0.1)
    assert ("lidar", "L1") not in paths._obs


def test_estimates_with_odom_matches_traj_shadow_transform():
    pose = {"x": 1.0, "y": -1.0, "theta": math.pi / 2}
    e = pp.estimates_with_odom([{"id": 3, "z": 2.0, "x": 0.5, "vx": 1.0, "vy": 0.0}], pose)[0]
    # base (2, -0.5) rotated 90 deg -> (0.5, 2) + (1, -1)
    assert (e["odom_x"], e["odom_y"]) == pytest.approx((1.5, 1.0))
    assert (e["odom_vx"], e["odom_vy"]) == pytest.approx((0.0, 1.0))


def test_v3_estimates_get_paths_without_odom_fields():
    paths = pp.PersonPaths("cv")
    out = None
    for i in range(15):
        est = [{"id": 7, "z": 2.0 + 0.1 * i, "x": 0.0, "vx": 1.0, "vy": 0.0}]
        out = pp.annotate_telemetry(paths, POSE0, [], [], est)[2]
    assert "path_rel" in out[0] and "odom_x" not in out[0]


@pytest.mark.skipif(not MODEL.exists(), reason="no deploy GRU")
def test_gru_mode_batches_tracks():
    paths = pp.PersonPaths("gru", MODEL)
    a = walker(1.79e9, 15, 0.1, 1.0, tid="L1")
    b = walker(1.79e9, 15, 0.1, 0.8, y0=2.0, th0=math.pi, tid="L2")
    out = None
    for i in range(15):
        out = paths.annotate([a[i], b[i]], "lidar", POSE0, now=100.0 + 0.1 * i, stamp_key="stamp")
    for r in out:
        p = np.array(r["path_rel"])
        assert p.shape == (pp.N_POINTS, 2) and np.isfinite(p).all()
        assert p[0].tolist() == [0.0, 0.0]
    assert np.array(out[0]["path_rel"])[-1, 0] > 1.0       # still heading forward
    assert np.array(out[1]["path_rel"])[-1, 0] < -0.8      # still heading back
    assert paths.stats["ms_max"] < 50


def test_gru_missing_model_falls_back_to_ctrv():
    p = pp.from_env("/nonexistent.npz", env="gru")
    assert p is not None and p.mode == "ctrv"


def test_fan_alternatives_bend_both_ways():
    path = np.column_stack([np.linspace(0, 2, 16), np.zeros(16)])
    t = np.linspace(0, 2, 16)
    left = pp.bend_path(path, t, 0.6, 0.7)
    right = pp.bend_path(path, t, -0.6, 0.7)
    assert left[-1, 1] > 0.2 and right[-1, 1] < -0.2
    arc = np.linalg.norm(np.diff(left, axis=0), axis=1).sum()
    assert arc == pytest.approx(2.0, abs=1e-6)                         # bending keeps arc length
