"""C3 Kalman tracker: static stays still, motion is recovered, gaps stay explicit."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import c3_person_tracker as pt


def cov(s=0.05):
    return np.diag([s * s, s * s])


def test_static_noisy_target_reports_low_speed():
    rng = np.random.default_rng(0)
    trk = pt.KalmanTracker({'meas_sigma_floor_m': 0.05})
    speeds = []
    for k in range(240):
        out = trk.update(k / 12, [(np.array([2.0, 0.5]) + rng.normal(0, 0.05, 2), cov())])
        speeds += [o['speed'] for o in out if o['id'] == 0]
    assert len(speeds) == 240
    # Gaussian 5 cm jitter alone still yields ~0.07 m/s median speed noise.
    assert np.median(speeds[60:]) < 0.1


def test_constant_velocity_is_recovered():
    trk = pt.KalmanTracker()
    for k in range(60):
        t = k / 12
        out = trk.update(t, [(np.array([1.0 + 0.8 * t, 0.2]), cov(0.02))])
    assert abs(out[0]['vx'] - 0.8) < 0.05 and abs(out[0]['vy']) < 0.05


def test_birth_needs_confirmation_and_miss_is_predicted_not_measured():
    trk = pt.KalmanTracker()
    out = trk.update(0.0, [(np.array([1.0, 0.0]), cov())])
    assert not out[0]['confirmed']
    for k in range(1, 4):
        out = trk.update(k * 0.1, [(np.array([1.0, 0.0]), cov())])
    assert out[0]['confirmed']
    out = trk.update(0.5, [])
    assert out[0]['measured'] is False and math.isclose(out[0]['obs_age_s'], 0.2)


def test_confirmed_track_expires_without_measurements():
    trk = pt.KalmanTracker({'confirmed_max_age_s': 0.3})
    for k in range(5):
        trk.update(k * 0.1, [(np.array([1.0, 0.0]), cov())])
    for k in range(5, 12):
        out = trk.update(k * 0.1, [])
    assert out == []


def test_gap_and_time_reversal_reset_instead_of_predicting():
    trk = pt.KalmanTracker()
    for k in range(5):
        trk.update(k * 0.1, [(np.array([1.0, 0.0]), cov())])
    assert trk.update(2.0, []) == []          # gap > max_dt_s
    trk.update(2.1, [(np.array([1.0, 0.0]), cov())])
    assert trk.update(1.0, []) == []          # stamp went backwards


def test_far_measurement_is_gated_into_a_new_track():
    trk = pt.KalmanTracker()
    for k in range(5):
        trk.update(k * 0.1, [(np.array([1.0, 0.0]), cov(0.02))])
    out = trk.update(0.5, [(np.array([3.0, 0.0]), cov(0.02))])
    assert len(out) == 2 and len({o['id'] for o in out}) == 2


def test_person_measurement_rejects_background_and_thin_support():
    depth = np.full((640, 480), 3.5, np.float32)
    depth[200:400, 200:280] = 1.5          # torso
    m = pt.person_measurement(depth, (180, 100, 300, 600))
    assert m is not None and abs(m['z'] - 1.5) < 1e-6
    assert pt.person_measurement(np.zeros((640, 480), np.float32), (180, 100, 300, 600)) is None


def test_camera_point_maps_forward_to_odom():
    # Optical z forward -> odom +x for a level camera at the origin.
    ofc = {'translation_xyz': [0.0, 0.0, 0.2], 'quaternion_xyzw': [-0.5, 0.5, -0.5, 0.5]}
    xy, rot = pt.camera_point_to_odom([0.0, 0.0, 2.0], ofc)
    assert np.allclose(xy, [2.0, 0.0], atol=1e-9)
    c = pt.rotate_cov_to_odom(np.diag([0.04, 0.01]), rot)
    assert np.allclose(c, np.diag([0.04, 0.01]), atol=1e-9)


def test_replay_depth_applies_recorded_device_correction():
    import c3_replay as rp
    arrays = {'depth': np.array([[0, 2440]], np.uint16)}
    corrected = rp.depth_metres(arrays, {'metadata': {'calibration': {'depth_inv_offset_per_m': 0.0821}}})
    assert corrected[0, 0] == 0 and abs(corrected[0, 1] - 2.04) < 0.01
    raw = rp.depth_metres(arrays, {'metadata': {'calibration': {}}})
    assert abs(raw[0, 1] - 2.44) < 1e-6
