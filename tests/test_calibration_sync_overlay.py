"""Synthetic checks for time_offset_xcorr.py and lidar_camera_overlay.py (no bag needed)."""
import math

import numpy as np
import pytest

import bag_signals as bs
import lidar_camera_overlay as ov
import time_offset_xcorr as xc


# ----------------------------------------------------------------- time offset

def _spin_profile(t):
    """Back-and-forth yaw rate with sharp reversals (what the protocol asks for)."""
    rng = np.random.default_rng(3)
    knots = np.arange(-2.0, 64.0, 1.3)          # fixed, so shifting t shifts the signal
    levels = rng.choice([-1.2, -0.8, 0.8, 1.2], size=len(knots))
    return np.interp(t, knots, levels)


@pytest.mark.parametrize('delay', [0.0, 0.023, -0.041, 0.120])
def test_xcorr_recovers_delay_low_rate_signal(delay):
    rng = np.random.default_rng(0)
    t_ref = np.arange(0, 60, 0.005)
    ref = _spin_profile(t_ref) + rng.normal(0, 0.05, t_ref.size)
    # 10 Hz signal: each sample is the mean rate over its 100 ms frame, stamped `delay` late
    t_true = np.arange(0.05, 60, 0.1)
    fine = np.arange(0, 60, 0.001)
    w = _spin_profile(fine)
    sig = np.array([w[(fine >= c - 0.05) & (fine < c + 0.05)].mean() for c in t_true])
    sig = -3.0 * sig + rng.normal(0, 0.05, sig.size)   # arbitrary gain and opposite sign
    r = xc.xcorr_offset(t_ref, ref, t_true + delay, sig, max_lag=0.3)
    assert r['sign'] == -1
    assert r['peak'] > 0.95
    assert abs(r['offset'] - delay) < 0.003


def test_xcorr_ignores_gaps():
    t_ref = np.arange(0, 60, 0.005)
    ref = _spin_profile(t_ref)
    t_sig = np.arange(0, 60, 0.02)
    keep = ~((t_sig > 20) & (t_sig < 26))           # a 6 s dropout
    t_sig = t_sig[keep]
    sig = _spin_profile(t_sig - 0.015)
    r = xc.xcorr_offset(t_ref, ref, t_sig, sig, max_lag=0.3)
    assert abs(r['offset'] - 0.015) < 0.002


def _room(rng, n=20000):
    """Points on the walls of a 6 x 4 m room plus a few boxes, sensor near (1, 0.5)."""
    pts = []
    for _ in range(n):
        side = rng.integers(4)
        u = rng.uniform()
        x, y = [(u * 6 - 2, -1.5), (u * 6 - 2, 2.5), (-2, u * 4 - 1.5), (4, u * 4 - 1.5)][side]
        pts.append((x, y, rng.uniform(-0.1, 1.0)))
    for cx, cy in [(2.0, 1.5), (-1.0, -1.0), (3.0, -0.8)]:
        for _ in range(1500):
            a = rng.uniform(0, 2 * np.pi)
            pts.append((cx + 0.25 * np.cos(a), cy + 0.25 * np.sin(a), rng.uniform(-0.1, 0.5)))
    return np.array(pts)


def test_profile_shift_recovers_yaw():
    rng = np.random.default_rng(1)
    room = _room(rng)
    yaw = math.radians(7.3)
    c, s = math.cos(-yaw), math.sin(-yaw)          # robot turns +yaw -> scene turns -yaw
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    a = xc.bearing_profile(room[rng.permutation(len(room))[:12000]])
    b = xc.bearing_profile(room[rng.permutation(len(room))[:12000]] @ R.T)
    k = xc.profile_shift(a, b, max_bins=400)
    assert abs(-k * 360 / 2880 - 7.3) < 0.3


def test_image_shift():
    import cv2
    rng = np.random.default_rng(2)
    img = cv2.GaussianBlur(rng.uniform(0, 255, (240, 360)).astype(np.float32), (7, 7), 2)
    dx, resp = xc.image_shift(img[:, 10:330], img[:, 4:324])
    assert abs(dx - 6) < 0.3 and resp > 0.2


def test_excitation_counts_reversals_not_noise():
    t = np.arange(0, 30, 0.005)
    rng = np.random.default_rng(4)
    still = xc.excitation(t, rng.normal(0, 0.05, t.size))
    spins = xc.excitation(t, _spin_profile(t))
    assert still['reversals'] == 0
    assert spins['reversals'] >= 6


# ----------------------------------------------------------------- overlay

def test_extrinsic_round_trip():
    for body in (False, True):
        p = [0.12, -0.03, 0.05, 1.5, -12.0, 88.0]
        q = ov.T_to_params(ov.params_to_T(p, body), body)
        assert np.allclose(p, q, atol=1e-9)


def test_body_frame_means_camera_looks_along_x():
    T = ov.params_to_T([0, 0, 0, 0, 0, 0], body_frame=True)
    p_cam = T[:3, :3] @ np.array([2.0, 0.0, 0.0]) + T[:3, 3]   # 2 m ahead of the lidar
    assert np.allclose(p_cam, [0, 0, 2.0])


def test_tf_lookup_chain():
    tfs = {('base_link', 'livox_frame'): bs.make_T(np.eye(3), [0.1, 0, 0.3]),
           ('base_link', 'cam_link'): bs.make_T(bs.rpy_to_mat(0, 0, math.pi / 2), [0, 0.2, 0.25])}
    T = bs.lookup(tfs, 'cam_link', 'livox_frame')
    p_base = np.array([0.1, 0, 0.3])                        # lidar origin in base
    p_cam = np.linalg.inv(tfs[('base_link', 'cam_link')]) @ np.r_[p_base, 1]
    assert np.allclose(T @ np.array([0, 0, 0, 1]), p_cam)


def _scene():
    """Lidar-frame points: a back wall at x=5, a box face at x=2.5; and a camera."""
    rng = np.random.default_rng(5)
    wall = np.column_stack([np.full(60000, 5.0), rng.uniform(-3, 3, 60000), rng.uniform(-1, 2, 60000)])
    # drop wall points the box hides from the lidar (it sits at the origin)
    hid = (np.abs(wall[:, 1] * 0.5) < 0.4) & (wall[:, 2] * 0.5 > -0.3) & (wall[:, 2] * 0.5 < 0.5)
    wall = wall[~hid]
    box = np.column_stack([np.full(20000, 2.5), rng.uniform(-0.4, 0.4, 20000), rng.uniform(-0.3, 0.5, 20000)])
    cam = ov.Camera([[400, 0, 320], [0, 400, 240], [0, 0, 1]], [0.05, -0.02, 0, 0, 0], 'plumb_bob', 640, 480)
    return np.vstack([wall, box]), cam


def _render_truth(cam, T):
    """Image the camera would see: dark wall, bright box (the box outline is the only edge)."""
    import cv2
    corners = np.array([[2.5, y, z] for y in (-0.4, 0.4) for z in (-0.3, 0.5)])
    uv, _, _ = cam.project(corners @ T[:3, :3].T + T[:3, 3])
    img = np.full((480, 640, 3), 40, np.uint8)
    hull = cv2.convexHull(uv.astype(np.float32)).astype(np.int32)
    cv2.fillConvexPoly(img, hull, (220, 220, 220))
    return img


def test_score_is_lowest_at_true_extrinsic():
    pts, cam = _scene()
    true = [0.05, 0.02, -0.04, 0.0, 0.0, 0.0]
    T = ov.params_to_T(true, body_frame=True)
    frame = {'image': _render_truth(cam, T), 'points': pts}
    s_true = ov.evaluate(frame, cam, T)[2]['score_px']
    off = list(true)
    off[5] += 1.5
    s_off = ov.evaluate(frame, cam, ov.params_to_T(off, True))[2]['score_px']
    assert s_true < 2.5                 # finite edge-band width: truth is ~1.8 px, not 0
    assert s_off > 2 * s_true


def test_sweep_pulls_yaw_and_pitch_back():
    pts, cam = _scene()
    true = [0.05, 0.02, -0.04, 0.0, 0.0, 0.0]
    frame = {'image': _render_truth(cam, ov.params_to_T(true, True)), 'points': pts}
    start = list(true)
    start[4] += 0.8
    start[5] -= 1.0
    dists = [ov.image_edge_distance(frame['image'])]
    best, _ = ov.sweep([frame], cam, start, True, dists)
    assert abs(best[4] - true[4]) < 0.25
    assert abs(best[5] - true[5]) < 0.25


# ----------------------------------------------------------------- end-to-end through a real bag

def _write_spin_bag(path, delay, duration=40.0):
    """Bag with a 200 Hz gyro and a 30 Hz camera panning over a periodic texture.

    Image header stamps are `delay` late. Exercises the whole camera branch of
    time_offset_xcorr: decode, downscale, phase correlation, midpoint stamps.
    """
    import cv2
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image, Imu
    from builtin_interfaces.msg import Time

    f = 250.0                                   # px per rad of the panorama
    width = int(round(2 * np.pi * f))
    rng = np.random.default_rng(7)
    tex = cv2.GaussianBlur(rng.uniform(0, 255, (120, width)).astype(np.float32), (9, 9), 2.5)
    fine = np.arange(0, duration + 1, 0.0005)
    theta = np.cumsum(_spin_profile(fine)) * 0.0005

    def stamp(t):
        return Time(sec=int(t), nanosec=int(round((t % 1) * 1e9)) % 1000000000)

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    w.create_topic(rosbag2_py.TopicMetadata(name='/imu', type='sensor_msgs/msg/Imu', serialization_format='cdr'))
    w.create_topic(rosbag2_py.TopicMetadata(name='/cam', type='sensor_msgs/msg/Image', serialization_format='cdr'))
    t0 = 100.0
    for t in np.arange(0.5, duration, 0.005):
        m = Imu()
        m.header.stamp = stamp(t0 + t)
        m.angular_velocity.z = float(_spin_profile(np.array([t]))[0] + rng.normal(0, 0.03))
        w.write('/imu', serialize_message(m), int((t0 + t) * 1e9))
    xs = np.arange(320, dtype=np.float32)
    for t in np.arange(0.5, duration, 1 / 30):
        shift = float(np.interp(t, fine, theta)) * f
        mx = np.tile(((xs + shift) % width)[None], (120, 1)).astype(np.float32)
        my = np.tile(np.arange(120, dtype=np.float32)[:, None], (1, 320))
        img = cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP).astype(np.uint8)
        m = Image()
        m.header.stamp = stamp(t0 + t + delay)
        m.height, m.width, m.encoding, m.step = 120, 320, 'mono8', 320
        m.data = img.tobytes()
        w.write('/cam', serialize_message(m), int((t0 + t + delay + 0.01) * 1e9))
    del w


def test_camera_branch_end_to_end(tmp_path):
    bag = tmp_path / 'spin'
    _write_spin_bag(bag, delay=0.025)
    rep = xc.main([str(bag), '--ref', '/imu', '--topics', '/cam', '--max-lag', '0.3'])
    cam = rep['topics']['/cam']
    assert cam['kind'] == 'Image'
    assert cam['peak_corr'] > 0.9
    assert abs(cam['offset_ms'] - 25.0) < 3.0
