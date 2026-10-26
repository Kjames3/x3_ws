import math
import random

from person_track_history import PersonTrackHistory, camera_to_robot


def _est(tid, rx, ry):
    """Readout estimate for a person at robot-frame (rx, ry)."""
    return {"id": tid, "x": -ry, "z": rx, "vx": 9.9, "vy": 9.9}


def _robot_frame(px, py, pyaw, mx, my):
    dx, dy = mx - px, my - py
    c, s = math.cos(pyaw), math.sin(pyaw)
    return dx * c + dy * s, -dx * s + dy * c


def test_camera_to_robot_axes():
    assert camera_to_robot({"x": 0.5, "z": 2.0}) == (2.0, -0.5)


def test_walker_velocity_with_the_robot_driving_and_turned():
    """Person crosses at 0.8 m/s in the map while the robot drives at 0.15 m/s."""
    h = PersonTrackHistory()
    pyaw = math.radians(30)
    for i in range(20):                       # 10 Hz for 2 s
        t = i * 0.1
        px, py = 0.15 * t * math.cos(pyaw), 0.15 * t * math.sin(pyaw)
        mx, my = 3.0, -1.0 + 0.8 * t
        h.update([_est(7, *_robot_frame(px, py, pyaw, mx, my))], (px, py, pyaw), t)
    vx, vy = h.velocity_map(7)
    assert abs(vx) < 0.02 and abs(vy - 0.8) < 0.02
    e = h.tracked([_est(7, 2.0, 0.0)], (0.0, 0.0, pyaw))[0]
    # Map +y seen from a robot yawed +30 deg: forward 0.8 sin30, left 0.8 cos30.
    assert abs(e["vx"] - 0.8 * math.sin(pyaw)) < 0.03
    assert abs(e["vy"] - 0.8 * math.cos(pyaw)) < 0.03
    assert e["vel_ok"] and e["vx_raw"] == 9.9


def test_position_noise_is_averaged_not_differentiated():
    random.seed(1)
    h = PersonTrackHistory()
    for i in range(10):                       # standing still, 5 cm noise
        h.update([_est(1, 2.0 + random.gauss(0, 0.05), random.gauss(0, 0.05))],
                 (0.0, 0.0, 0.0), i * 0.1)
    vx, vy = h.velocity_map(1)
    assert math.hypot(vx, vy) < 0.25          # single-frame differencing gives ~0.7


def test_repeated_readouts_are_not_new_observations():
    """The readout repeats an estimate until the estimator updates; the robot
    driving meanwhile must not turn the repeat into person motion."""
    h = PersonTrackHistory()
    for i in range(10):
        t = i * 0.1
        h.update([_est(3, 2.0, 0.0)], (0.15 * t, 0.0, 0.0), t)
    assert h.velocity_map(3) is None          # one real observation only


def test_short_history_reports_zero_velocity():
    h = PersonTrackHistory()
    h.update([_est(4, 2.0, 0.0)], (0.0, 0.0, 0.0), 0.0)
    h.update([_est(4, 2.1, 0.0)], (0.0, 0.0, 0.0), 0.1)
    e = h.tracked([_est(4, 2.1, 0.0)], (0.0, 0.0, 0.0))[0]
    assert (e["vx"], e["vy"], e["vel_ok"]) == (0.0, 0.0, False)


def test_new_id_near_a_lost_track_inherits_its_history():
    h = PersonTrackHistory()
    for i in range(6):
        h.update([_est(10, 3.0 - 0.1 * i, 0.0)], (0.0, 0.0, 0.0), i * 0.1)
    # ID churn: same person, new ID, next readout.
    h.update([_est(11, 2.4, 0.0)], (0.0, 0.0, 0.0), 0.65)
    vx, _ = h.velocity_map(11)
    assert abs(vx + 1.0) < 0.1
    assert h.velocity_map(10) is None


def test_far_new_id_starts_fresh_and_lost_tracks_expire():
    h = PersonTrackHistory()
    for i in range(6):
        h.update([_est(20, 3.0, 0.0)], (0.0, 0.0, 0.0), i * 0.1)
    h.update([_est(21, 1.0, 1.5)], (0.0, 0.0, 0.0), 0.6)
    assert h.velocity_map(21) is None
    h.update([_est(21, 1.0, 1.5)], (0.0, 0.0, 0.0), 2.0)
    assert 20 not in h._tracks
