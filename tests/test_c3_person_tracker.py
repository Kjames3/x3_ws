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


# --- depth read from a person box: clipping and the histogram mode ----------

def _scene(rows=640, cols=480, wall=3.5):
    return np.full((rows, cols), wall, np.float32)


def test_in_image_box_is_unchanged_by_clip_flag_and_window_is_default():
    depth = _scene()
    depth[200:400, 200:280] = 1.5
    box = (180, 100, 300, 600)
    a = pt.person_measurement(depth, box)
    b = pt.person_measurement(depth, box, clip_box=False, depth_mode='window')
    assert a == b


def test_person_in_front_of_wall_both_modes_read_the_person():
    depth = _scene()
    depth[150:520, 190:290] = 1.8            # body, 100 px wide, 370 px tall
    box = (180, 140, 300, 530)
    for mode in pt.DEPTH_MODES:
        m = pt.person_measurement(depth, box, depth_mode=mode)
        assert m is not None and abs(m['z'] - 1.8) < 0.02, mode


def test_box_above_image_top_is_clipped_before_the_torso_window():
    # Seated person: head and shoulders are above the frame, the visible part is
    # rows 0-250. The detector box starts 400 px above the top, so the unclipped
    # 20-55 % window (rows -320..-180 .. 0) lands on the partition behind them.
    depth = _scene(wall=3.4)
    depth[0:250, 190:290] = 1.6
    box = (180, -400, 300, 260)
    old = pt.person_measurement(depth, box, clip_box=False)
    assert old is None or old['z'] > 3.0      # window sits off-image or on the wall
    new = pt.person_measurement(depth, box)
    assert new is not None and abs(new['z'] - 1.6) < 0.02
    assert 0 <= new['v'] <= 250               # placed inside the image


def test_box_off_the_left_edge_is_clipped_too():
    depth = _scene()
    depth[200:500, 0:60] = 2.0
    box = (-300, 150, 80, 520)               # most of the box is off-image
    new = pt.person_measurement(depth, box)
    assert new is not None and abs(new['z'] - 2.0) < 0.02
    assert pt.person_measurement(depth, (-300, 150, -10, 520)) is None   # nothing in frame


def test_nearest_mode_ignores_background_that_dominates_the_window():
    # Torso window mostly background: the person only fills the left third of the
    # central columns, so the median reads the wall. The histogram's nearest
    # substantial peak is the person.
    depth = _scene(wall=3.5)
    depth[100:500, 215:235] = 2.2            # narrow person
    box = (180, 100, 300, 500)
    w = pt.person_measurement(depth, box, depth_mode='window')
    assert abs(w['z'] - 3.5) < 0.02
    h = pt.person_measurement(depth, box, depth_mode='nearest_mode')
    assert abs(h['z'] - 2.2) < 0.02
    assert 215 <= h['u'] <= 235              # u is the centroid of the person pixels


def test_nearest_mode_with_desk_edge_in_front_at_lower_height():
    # A desk edge 1.0 m closer fills the bottom 30 % of the box. The histogram
    # reads only the upper rows, so it is not the nearest substantial peak.
    depth = _scene(wall=3.5)
    depth[100:500, 190:290] = 2.0            # person
    depth[400:500, 150:330] = 1.0            # desk lip in front of the legs
    box = (180, 100, 300, 500)
    m = pt.person_measurement(depth, box, depth_mode='nearest_mode')
    assert abs(m['z'] - 2.0) < 0.02


def test_nearest_mode_clipped_top_box_and_sloped_person():
    depth = _scene(wall=3.0)
    rows = np.arange(0, 300)[:, None]
    depth[0:300, 190:290] = (1.4 + 0.0004 * rows).astype(np.float32)   # leaning surface, ~12 cm over the box
    box = (180, -500, 300, 320)
    m = pt.person_measurement(depth, box, depth_mode='nearest_mode')
    assert m is not None and abs(m['z'] - 1.45) < 0.08


def test_nearest_mode_rejects_thin_noise_and_empty_depth():
    rng = np.random.default_rng(1)
    depth = rng.uniform(0.6, 3.9, (640, 480)).astype(np.float32)   # no peak holds 15 %
    assert pt.person_measurement(depth, (180, 100, 300, 500), depth_mode='nearest_mode') is None
    assert pt.person_measurement(np.zeros((640, 480), np.float32), (180, 100, 300, 500),
                                 depth_mode='nearest_mode') is None


def test_nearest_mode_ground_mask_drops_floor_peak():
    depth = _scene(wall=3.5)
    depth[100:500, 190:290] = 2.0
    depth[300:420, 190:290] = 1.2            # a floor patch that forms a real peak
    ground = np.zeros(depth.shape, bool)
    ground[300:420, :] = True
    box = (180, 100, 300, 500)
    assert abs(pt.person_measurement(depth, box, depth_mode='nearest_mode')['z'] - 1.2) < 0.02
    m = pt.person_measurement(depth, box, depth_mode='nearest_mode', ground_mask=ground)
    assert abs(m['z'] - 2.0) < 0.02


def test_unknown_depth_mode_raises():
    import pytest
    with pytest.raises(ValueError):
        pt.person_measurement(_scene(), (0, 0, 10, 10), depth_mode='bogus')
