"""side_person_rays: side-camera pixel -> odom ray geometry on the real calibration files."""
import math
import pathlib
import sys

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src' / 'yahboomcar_bringup')]

import side_person_rays as spr  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

LIVOX_IN_BASE = np.array([-0.0025, 0.0003, 0.302])     # v2 chassis, livox_frame in base_footprint


def chain(frame):
    """A /tf_static stand-in that only has the Livox (exercises the JSON fallback)."""
    return (np.eye(3), LIVOX_IN_BASE) if frame == 'livox_frame' else None


def _cam(role, frame_wh=None):
    c = spr.load_camera(role)
    wh = frame_wh or c['wh']
    K = spr.scaled_K(c['K'], c['wh'], wh)
    R, t = spr.camera_in_base(c, chain)
    return c, K, wh, R, t


def test_fields_match_person_candidates():
    assert spr.RAY_IN_FIELDS == pc.RAY_IN_FIELDS
    assert spr.RANGE_M == pc.RAY_RANGE_M


@pytest.mark.parametrize('role,side', [('left', 1.0), ('right', -1.0)])
def test_optical_axis_points_sideways_and_slightly_up(role, side):
    c, K, wh, R, t = _cam(role)
    d = R @ spr.pixel_dirs(K, c['D'], [(K[0, 2], K[1, 2])])[0]
    assert side * d[1] > 0.95                                   # left: +y, right: -y
    assert 0.05 < d[2] < 0.35                                    # ~9-10 deg nose up
    assert 0.15 < t[2] < 0.3                                     # ~0.2 m above the floor
    # image right is robot forward for the left camera, robot backward for the right one
    right = R @ spr.pixel_dirs(K, c['D'], [(K[0, 2] + 200 * wh[0] / 1280, K[1, 2])])[0]
    assert side * (right[0] - d[0]) > 0.1


@pytest.mark.parametrize('role', ['left', 'right'])
def test_pixel_dirs_round_trip_projectpoints(role):
    c, K, wh, _, _ = _cam(role)
    rng = np.random.default_rng(0)
    p = np.c_[rng.uniform(-1.2, 1.2, 50), rng.uniform(-0.7, 0.7, 50), np.ones(50)]
    uv, _ = cv2.fisheye.projectPoints(p.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, c['D'])
    d = spr.pixel_dirs(K, c['D'], uv.reshape(-1, 2))
    assert np.allclose(d, p / np.linalg.norm(p, axis=1, keepdims=True), atol=1e-4)


def test_left_binned_frame_scales_K_only():
    c = spr.load_camera('left')
    K = spr.scaled_K(c['K'], c['wh'], (640, 400))
    assert K[0, 0] == pytest.approx(c['K'][0, 0] / 2) and K[1, 1] == pytest.approx(c['K'][1, 1] / 2)
    # the same direction lands on the same (halved) pixel
    d = spr.pixel_dirs(K, c['D'], [(320.0, 200.0)])[0]
    d_full = spr.pixel_dirs(c['K'], c['D'], [(640.5, 400.5)])[0]
    assert np.allclose(d, d_full, atol=1e-6)


def test_urdf_frame_preferred_over_json():
    c = spr.load_camera('left')
    R0, t0 = np.eye(3), np.array([1.0, 2.0, 3.0])
    assert spr.camera_in_base(c, lambda f: (R0, t0) if f == c['frame'] else None)[1] is t0
    assert spr.camera_in_base(c, lambda f: None) is None


def _box_of_person(role, pose, px, py, top=1.75, frame_wh=None, r=0.2):
    """Project a standing person (cylinder, feet at z=0) at odom (px, py) into the camera."""
    c, K, wh, R, t = _cam(role, frame_wh)
    cth, sth = math.cos(pose['theta']), math.sin(pose['theta'])
    R_ob = np.array([[cth, -sth, 0], [sth, cth, 0], [0, 0, 1]])
    a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    pts = [(px + r * math.cos(q), py + r * math.sin(q), z) for q in a for z in (0.0, top)]
    po = np.array(pts) - [pose['x'], pose['y'], 0.0]
    pc_ = (R.T @ (R_ob.T @ po.T - t[:, None])).T                 # optical frame
    assert (pc_[:, 2] > 0).all()
    uv, _ = cv2.fisheye.projectPoints(pc_.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, c['D'])
    uv = uv.reshape(-1, 2)
    return (uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()), c, K, wh, R, t


@pytest.mark.parametrize('role,frame_wh,rel', [('left', (640, 400), (0.4, 3.0)),
                                               ('right', None, (-0.6, -2.5))])
def test_detection_ray_passes_through_the_person(role, frame_wh, rel):
    pose = {'x': 1.0, 'y': -2.0, 'theta': 0.4}
    c0, s0 = math.cos(pose['theta']), math.sin(pose['theta'])
    px, py = pose['x'] + c0 * rel[0] - s0 * rel[1], pose['y'] + s0 * rel[0] + c0 * rel[1]
    box, c, K, wh, R, t = _box_of_person(role, pose, px, py, frame_wh=frame_wh)
    ray = spr.detection_ray(box, wh, K, c['D'], R, t, pose)
    dx, dy = px - ray['ox'], py - ray['oy']
    s = dx * ray['ux'] + dy * ray['uy']
    lat = -dx * ray['uy'] + dy * ray['ux']
    assert abs(lat) < 0.1                                        # bearing through the body
    assert ray['prior'] == 'feet+height' and ray['s_lo'] < s < ray['s_hi']
    assert ray['s_hi'] - ray['s_lo'] < 4.0
    # the box top implies the head height at the person's (front-surface) range
    assert ray['oz'] + (s - 0.2) * ray['top_tan'] == pytest.approx(1.75, abs=0.12)


def test_cut_off_box_gives_no_top_and_the_full_ray():
    c, K, wh, R, t = _cam('right')
    ray = spr.detection_ray((500, 0, 700, wh[1] - 1), wh, K, c['D'], R, t, {'x': 0, 'y': 0, 'theta': 0})
    assert math.isnan(ray['top_tan']) and math.isnan(ray['bot_tan'])
    assert (ray['s_lo'], ray['s_hi'], ray['prior']) == (spr.RANGE_M[0], spr.RANGE_M[1], 'none')
    assert spr.detection_ray((500, 300, 700, 310), wh, K, c['D'], R, t, {'x': 0, 'y': 0, 'theta': 0}) is None


def test_range_window():
    lo, hi, kind = spr.range_window(0.2, float('nan'), -0.2 / 3.0)       # feet at 3 m
    assert kind == 'feet' and lo == pytest.approx(1.8) and hi == pytest.approx(5.1)
    lo, hi, kind = spr.range_window(0.2, 0.25, float('nan'))             # no bottom: nothing
    assert (lo, hi, kind) == (0.4, 6.0, 'none')
    lo, hi, kind = spr.range_window(0.2, 0.5, 0.1)                       # legs hidden, short box
    assert kind == 'height' and hi == pytest.approx(2.1 / 0.4 + 0.3) and lo == 0.4


def test_side_track_ids():
    tr = spr.SideTrackIds()
    a, b = (100, 100, 200, 400), (600, 100, 700, 400)
    assert [r[2] for r in tr.update([a], 0.0)] == [False]            # one sighting: not ready
    out = tr.update([(110, 100, 210, 400), b], 0.2)
    assert out[0][0] == 1 and out[0][2] and out[1][0] == 2 and not out[1][2]
    out = tr.update([b], 0.7)
    assert out[0][0] == 2 and out[0][2]
    out = tr.update([(105, 100, 205, 400)], 1.0)                     # id 1 unseen 0.8 s: expired
    assert out[0][0] == 3 and not out[0][2]


def test_rear_pinhole_camera_rays_point_backwards():
    """The rear OAK uses the rational (non-fisheye) model and looks behind the robot."""
    import numpy as np
    import side_person_rays as spr
    cam = spr.load_camera('rear')
    assert cam['D'].shape == (14,) and cam['wh'] == (1280, 720)
    d = spr.pixel_dirs(cam['K'], cam['D'], [(640, 360), (0, 360), (1279, 360)])
    yaw = np.degrees(np.arctan2(d[:, 0], d[:, 2]))
    assert abs(yaw[0]) < 2 and 40 < -yaw[1] < 55 and 40 < yaw[2] < 55
    R_bl, t_bl = np.eye(3), np.zeros(3)
    R, t = R_bl @ cam['R_lc'], cam['t_lc']
    ray = spr.detection_ray((600, 200, 680, 600), (1280, 720), cam['K'], cam['D'], R, t,
                            {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    assert ray is not None and ray['ux'] < -0.95     # centre of the image = behind (-x)
