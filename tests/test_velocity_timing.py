"""Timing regressions: geometry, irregular cadence, duplicates, and outages."""
import math
import sys
import threading
from collections import deque
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from velocity_timing import PoseHistory, resample_positions, ros_stamp_ns
from velocity_estimator import VelocityEstimator
from oakd_driver import OakDCamera


def test_irregular_sampling_recovers_constant_velocity_at_training_cadence():
    ts = np.array([0, .08, .19, .30, .44, .51, .64, .73, .86, .98, 1.09])
    samples = [(t, (2 + .6*t, -.3*t, 2)) for t in ts]
    window = resample_positions(samples)
    assert window.shape == (10, 3)
    np.testing.assert_allclose(np.diff(window[:, :2], axis=0),
                               np.tile([.06, -.03], (9, 1)), atol=1e-12)


def test_resampling_never_pads_or_bridges_outage():
    assert resample_positions([(0, (0, 0, 0)), (.2, (1, 0, 0))]) is None
    ts = [0, .1, .2, .6, .7, .8, .9, 1.0]
    assert resample_positions([(t, (t, 0, 0)) for t in ts]) is None
    assert resample_positions([(0, (0, 0, 0)), (0, (0, 0, 0))]) is None


def test_pose_interpolates_across_yaw_wrap_and_rejects_extrapolation():
    h = PoseHistory()
    h.add(1, 0, 0, math.radians(179))
    h.add(1.1, .1, 0, math.radians(-179))
    p = h.at(1.05)
    assert p['x'] == pytest.approx(.05)
    assert abs(p['theta']) == pytest.approx(math.pi)
    assert h.at(.99) is None
    assert h.at(1.11) is None
    h.add(2, 1, 0, 0)
    assert h.at(1.5) is None


def test_capture_pose_keeps_static_world_point_stationary():
    h = PoseHistory()
    for t in np.arange(0, 2, .05):
        h.add(t, .4*t, 0, .6*t)
    world = np.array([2., 1.])
    for t in np.arange(.1, 1.7, .1):
        pose = h.at(t)
        theta = pose['theta']
        rotation = np.array([[math.cos(theta), -math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])
        local = rotation.T @ (world - [pose['x'], pose['y']])
        np.testing.assert_allclose(rotation @ local + [pose['x'], pose['y']], world)
        # Pairing the same capture with a pose 200 ms later changes its position.
        later = h.at(t + .2)
        assert abs(later['x'] - pose['x']) > .07


def estimator():
    return VelocityEstimator(None, None, model_path=str(
        Path(__file__).resolve().parents[1] / 'src/velocity_mlp_v3.torchscript'))


def test_step_duplicate_does_not_advance_tracker_or_window_and_gap_resets():
    est = estimator()
    for t in np.arange(0, 1.11, .1):
        estimates, tracks = est._step([(0, 0, 2 + .3*t)], 0, 0, 0, capture_t=float(t))
    assert estimates
    tid = next(iter(tracks))
    count = est._tracker.tracks[tid]['visible_count']
    n = len(est._timed_histories[tid])
    assert est._step([(0, 0, 3)], 0, 0, 0, capture_t=float(t)) == ([], {})
    assert est._tracker.tracks[tid]['visible_count'] == count
    assert len(est._timed_histories[tid]) == n
    out, tracks = est._step([(0, 0, 2.5)], 0, 0, 0, capture_t=2.)
    assert not out
    assert next(iter(tracks)) != tid
    assert est.timing_stats['gap_resets'] == 1


def test_unmatched_track_cannot_produce_new_timed_velocity():
    est = estimator()
    for t in np.arange(0, 1.11, .1):
        est._step([(0, 0, 2 + .3*t)], 0, 0, 0, capture_t=float(t))
    out, _ = est._step([], 0, 0, 0, capture_t=1.2)
    assert out == []


def test_atomic_depth_snapshot_retains_matching_pixels_and_metadata():
    cam = OakDCamera.__new__(OakDCamera)
    cam._lock = threading.Lock()
    cam._depth_history = deque(maxlen=20)
    cam._inv_depth_offset = 0.
    cam._depth_intr_size = (3, 2)
    cam._depth_fx = cam._depth_fy = 100.
    cam._depth_cx = cam._depth_cy = 1.
    cam._process_depth(np.full((2, 3), 1000, np.uint16), 10., ('a', 1), 10.1)
    first = cam.get_depth_observation()
    cam._process_depth(np.full((2, 3), 2000, np.uint16), 10.1, ('a', 2), 10.2)
    second = cam.get_depth_observation()
    assert first['frame_id'] == ('a', 1) and first['capture_t'] == 10.
    assert np.all(first['depth'] == 1.)
    assert np.all(second['depth'] == 2.)
    assert first['intrinsics'][-2:] == (3, 2)


@pytest.mark.parametrize('pose_available', [True, False])
def test_live_loop_uses_capture_pose_and_deduplicates(monkeypatch, pose_available):
    import velocity_estimator as ve
    now = [10.1]
    calls = []
    pose_calls = []
    class Camera:
        def get_depth_observation(self):
            return {'capture_t': 10., 'frame_id': ('session', 7),
                    'receipt_t': 10.05, 'depth': np.full((4, 4), 2.),
                    'intrinsics': (100, 100, 2, 2, 4, 4)}
    est = estimator()
    est.camera = Camera()
    def latest_pose():
        raise AssertionError('must not substitute current pose')
    est.robot_pose_fn = latest_pose
    est.robot_pose_at_fn = lambda t: pose_calls.append(t) or (
        {'x': 1., 'y': 2., 'theta': .3} if pose_available else None)
    est._extract_depth_centroids = lambda *args: [(0, 0, 2)]
    original = est._step
    def step(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)
    est._step = step
    est._running = True
    def sleep(_):
        now[0] += .1
        if now[0] >= 10.3:
            est._running = False
    monkeypatch.setattr(ve.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(ve.time, 'sleep', sleep)
    monkeypatch.delenv('DISPLAY', raising=False)
    est._inference_loop()
    if pose_available:
        assert len(calls) == 1
        assert calls[0][0][1:4] == (1., 2., .3)
        assert calls[0][1]['capture_t'] == 10.
        assert est.timing_stats['duplicates'] >= 1
    else:
        assert not calls
        assert est.timing_stats['missing_pose'] == 1


def test_uniform_timed_window_matches_legacy_features():
    est = estimator()
    est._feature_log_path = Path('/unused')
    for t in np.arange(10) * .1:
        est._step([(0, 0, 2 + .5*t)], 0, 0, 0, capture_t=float(t))
    timed = next(r['feats'] for r in reversed(est._feature_rows) if r['status'] == 'ok')
    legacy, _ = est._build_window_features([(0, 0, 2 + .5*t) for t in np.arange(10)*.1])
    np.testing.assert_allclose(timed, legacy.ravel(), atol=3e-7)


def _walk_then_flicker(est, flicker_s, t_walk=1.5):
    """Person walking toward the robot at 1 m/s, then no centroid for flicker_s."""
    last = None
    for t in np.arange(0, t_walk, .1):
        out, _ = est._step([(0, 0, 3 - t)], 0, 0, 0, [True], capture_t=float(t))
        last = out
    return last, t_walk - .1


def test_matched_track_coasts_on_last_velocity_while_the_window_rebuilds():
    est = estimator()
    out, t = _walk_then_flicker(est, 0)
    assert out and not out[0].get('coasted')
    vx = out[0]['vx']
    # centroid missing for 0.4 s of frames (> MAX_SAMPLE_GAP_S) then back: history clears
    for _ in range(4):
        t += .1
        est._step([], 0, 0, 0, [], capture_t=float(t))
    t += .1
    out, _ = est._step([(0, 0, 3 - t)], 0, 0, 0, [True], capture_t=float(t))
    assert out and out[0]['coasted'] and out[0]['vx'] == vx
    assert est.timing_stats['coasted'] >= 1 and est.timing_stats['window_wait'] >= 1


def test_coasting_stops_after_coast_max():
    est = estimator()
    out, t = _walk_then_flicker(est, 0)
    for _ in range(4):
        t += .1
        est._step([], 0, 0, 0, [], capture_t=float(t))
    # matched again but the window is still rebuilding: keep matching for > COAST_MAX_S
    coasted = []
    for k in range(1, 20):
        t += .1
        out, _ = est._step([(0, 0, 3 - t)], 0, 0, 0, [True], capture_t=float(t))
        coasted.append(any(e.get('coasted') for e in out))
    assert coasted[0] and not coasted[-1]   # coasts at first, then the window is ready or coasting expired
    assert est.timing_stats['coasted'] <= 15


def test_never_coasts_without_a_prior_model_output_or_for_non_people():
    est = estimator()
    out, _ = est._step([(0, 0, 2)], 0, 0, 0, [True], capture_t=0.)
    assert not any(e.get('coasted') for e in out)
    est = estimator()
    for t in np.arange(0, 1.5, .1):
        est._step([(0, 0, 3 - t)], 0, 0, 0, [False], capture_t=float(t))
    out, _ = est._step([(0, 0, 1.4)], 0, 0, 0, [False], capture_t=1.9)
    assert not any(e.get('coasted') for e in out)


def test_ros_stamp_places_capture_at_its_age():
    assert ros_stamp_ns(10_000_000_000, 50.0, 49.7) == 9_700_000_000
    assert ros_stamp_ns(10_000_000_000, 50.0, 50.2) is None      # from the future
    assert ros_stamp_ns(10_000_000_000, 50.0, 40.0) is None      # implausibly old
    assert ros_stamp_ns(10_000_000_000, 50.0, float('nan')) is None
    assert ros_stamp_ns(10_000_000_000, 50.0, None) is None
