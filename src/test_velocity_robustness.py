"""Regression tests for velocity-estimator input-validity handling (item 8).

These cover three failure modes that were all silent: a stalled depth camera, a
missing robot pose, and a contour with no valid depth. Each one used to produce
plausible-looking obstacle output rather than an error.
"""

import os
import sys
import threading
import time

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import velocity_estimator as ve  # noqa: E402
from velocity_estimator import ObstacleTracker, VelocityEstimator  # noqa: E402


class _FakeCamera:
    """Camera whose depth frame is fixed and whose age we control."""

    def __init__(self, age=0.0):
        self.age = age
        self.frame = np.full((80, 120), 2.0, dtype=np.float32)

    def get_depth_frame(self):
        return self.frame

    def get_raw_depth_frame(self):
        return self.frame

    def get_depth_intrinsics(self):
        return (300.0, 300.0, 60.0, 40.0, 120, 80)

    def get_depth_frame_age(self):
        return self.age


def _loop_estimator(camera, robot_pose_fn=None, pose_fn_configured=False):
    """A VelocityEstimator wired for one pass of _inference_loop, no model."""
    est = VelocityEstimator.__new__(VelocityEstimator)
    est.camera = camera
    est.lidar = None
    est.robot_pose_fn = robot_pose_fn
    est.detections_fn = None
    est.estimation_enabled = False
    est._model = None
    est._tracker = ObstacleTracker()
    est._estimates = []
    est._lock = threading.Lock()
    est._running = True
    est._last_print_time = 0.0
    est._prev_estimates = {}
    est._logged_missing_intrinsics = False
    est._logged_stale_depth = False
    est._logged_missing_pose = False
    est._height_ray_cache = {}
    est.camera_height_m = 0.210
    est.camera_pitch_rad = 0.0
    est.min_obstacle_height_m = 0.15
    est.max_obstacle_height_m = 2.0
    return est


def _run_one_pass(est):
    """Run _inference_loop for a single iteration."""
    def stop_after_one():
        time.sleep(0.25)
        est._running = False
    t = threading.Thread(target=stop_after_one)
    t.start()
    est._inference_loop()
    t.join()


# --------------------------------------------------------------- stale frames

def test_stale_depth_frame_is_rejected():
    cam = _FakeCamera(age=ve.MAX_DEPTH_FRAME_AGE_S + 1.0)
    est = _loop_estimator(cam)
    est._tracker.tracks = {0: {"centroid": (0.0, 0.0, 1.0)}}
    _run_one_pass(est)
    assert est.get_estimates() == []
    # Tracks are dropped so the pre-gap history cannot be spliced onto post-gap
    # frames once the camera recovers.
    assert est._tracker.tracks == {}


def test_fresh_depth_frame_is_not_rejected():
    cam = _FakeCamera(age=0.0)
    est = _loop_estimator(cam)
    _run_one_pass(est)
    # A fresh frame must reach extraction; the flat 2.0 m wall is inside the
    # height band, so it produces at least one centroid.
    assert est._logged_stale_depth is False


def test_staleness_threshold_is_a_real_bound():
    # Guards against the constant being set to something that never trips or
    # always trips.
    assert 0.1 < ve.MAX_DEPTH_FRAME_AGE_S < 5.0


# ------------------------------------------------------------- missing odometry

def test_missing_pose_skips_frame_when_odometry_is_configured():
    cam = _FakeCamera(age=0.0)

    def broken_pose():
        raise RuntimeError("odom down")

    est = _loop_estimator(cam, robot_pose_fn=broken_pose)
    est._tracker.tracks = {0: {"centroid": (0.0, 0.0, 1.0)}}
    _run_one_pass(est)
    assert est.get_estimates() == []
    assert est._tracker.tracks == {}
    assert est._logged_missing_pose is True


def test_pose_returning_no_pose_key_skips_frame():
    cam = _FakeCamera(age=0.0)
    est = _loop_estimator(cam, robot_pose_fn=lambda: {"twist": {}})
    _run_one_pass(est)
    assert est.get_estimates() == []
    assert est._logged_missing_pose is True


def test_absent_pose_fn_uses_robot_frame_without_warning():
    # robot_pose_fn=None is a legitimate configuration (robot-local frame), not
    # a failure, and must not be treated as missing odometry.
    cam = _FakeCamera(age=0.0)
    est = _loop_estimator(cam, robot_pose_fn=None)
    _run_one_pass(est)
    assert est._logged_missing_pose is False


# ----------------------------------------------------------- no-depth contours
#
# The Z = 1.0 fallback was removed from _extract_depth_centroids, but NO test
# asserts it here on purpose: the branch is unreachable in the raw-depth path.
# Every contour is traced from mask pixels that are in [0.5, 4.0] by
# construction, and filling a contour only adds pixels (holes, MORPH_CLOSE), so
# valid_depths cannot come back empty. An adversarial thin valid ring around a
# large invalid hole still yields thousands of valid samples. The removal is
# hardening against a future change to how the mask is built -- e.g. the
# percentile-depth or wall filters still on the item 8 list -- not a fix for
# observed behaviour. A test here would pass identically before and after.


# ------------------------------------------------------- configurable range gate

def _config_estimator(tmp_path, monkeypatch, **overrides):
    cfg = {
        "camera_height_m": 0.21,
        "camera_pitch_deg": 1.077,
        "min_obstacle_height_m": 0.15,
        "max_obstacle_height_m": 2.0,
    }
    cfg.update(overrides)
    path = tmp_path / "camera_ground_plane.json"
    path.write_text(__import__("json").dumps(cfg))
    monkeypatch.setattr(ve, "GROUND_PLANE_CONFIG_PATH", str(path))
    est = VelocityEstimator.__new__(VelocityEstimator)
    est._load_ground_plane_config()
    return est


def test_speed_range_defaults_to_the_extraction_ceiling(tmp_path, monkeypatch):
    # This asserted 1.8 while the safety gate lived inside the estimator.
    # The gate moved to the CBF boundary, so the estimator now reports the full
    # range it can measure and 1.8 would silently truncate telemetry again.
    est = _config_estimator(tmp_path, monkeypatch)
    assert est.max_speed_range_m == ve.DEFAULT_MAX_SPEED_RANGE_M == 4.0


def test_speed_range_is_configurable(tmp_path, monkeypatch):
    est = _config_estimator(tmp_path, monkeypatch, max_speed_range_m=4.0)
    assert est.max_speed_range_m == 4.0


@pytest.mark.parametrize("bad", [0.0, -1.0, 4.5])
def test_speed_range_out_of_bounds_is_rejected(tmp_path, monkeypatch, bad):
    # 0 or negative silently disables all speed estimation; beyond 4.0 m is past
    # the depth extraction ceiling and can never match a centroid.
    with pytest.raises(ValueError):
        _config_estimator(tmp_path, monkeypatch, max_speed_range_m=bad)


def test_gate_reads_config_and_no_hardcoded_range_remains():
    # The gate used to be the literal `track['centroid'][2] > 1.8`. Assert the
    # comparison now reads the configured attribute, so raising the range in
    # config cannot silently leave a second hardcoded gate behind.
    import inspect
    src = inspect.getsource(VelocityEstimator._inference_loop)
    assert "self.max_speed_range_m" in src
    assert "> 1.8" not in src


# ------------------------------------------- estimator vs CBF range separation

def test_estimator_default_range_is_the_extraction_ceiling():
    # The estimator reports everything it can measure. The 1.8 m limit is a
    # SAFETY parameter and now lives at the CBF boundary, not here.
    assert ve.DEFAULT_MAX_SPEED_RANGE_M == 4.0


def test_cbf_gate_is_separate_and_still_defaults_to_1_8():
    # Reading the constant from source keeps this test independent of importing
    # server_x3, which pulls in ROS and hardware drivers.
    import re
    from pathlib import Path
    src = (Path(__file__).parent / "server_x3.py").read_text()
    match = re.search(r"^CBF_SPEED_RANGE_M\s*=\s*([\d.]+)", src, re.M)
    assert match, "CBF_SPEED_RANGE_M must exist: it is the safety gate"
    assert float(match.group(1)) == 1.8


def test_cbf_rejects_far_obstacles_before_the_speed_test():
    # The order matters. A far track now arrives with a REAL speed instead of
    # 0.0, so the range check must come first or fast far-field noise would
    # reach the CBF -- exactly what the old estimator-side gate prevented.
    from pathlib import Path
    src = (Path(__file__).parent / "server_x3.py").read_text()
    body = src[src.index("estimates = velocity_estimator.get_estimates()"):]
    body = body[:body.index("except Exception")]
    assert body.index("CBF_SPEED_RANGE_M") < body.index("speed > 0.15")
