"""VL53L5CX geometry, status filtering and node plumbing — all without hardware.

The things this actually guards are the ones that bite silently: a status
filter that lets garbage through, a zone reordering that mirrors the world, and
an empty frame being mistaken for a clear path.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'yahboomcar_bringup'))
sys.path.insert(0, str(ROOT / 'src'))

from yahboomcar_bringup.tof_geometry import (  # noqa: E402
    DEFAULT_MAX_RANGE_M, FOV_DEG, VALID_STATUS, floor_coverage,
    frame_to_points, ray_table, reorder, valid_mask)

N = 8
N2 = N * N


def frame(dist_mm=1000.0, status=5):
    return (np.full(N2, float(dist_mm)), np.full(N2, int(status), dtype=np.int32))


# --------------------------------------------------------------- ray table

def test_rays_are_unit_and_centred():
    r = ray_table(N)
    assert r.shape == (N2, 3)
    assert np.allclose(np.linalg.norm(r, axis=1), 1.0)
    # Symmetric about the optical axis: the mean ray points straight ahead.
    assert np.allclose(r.mean(axis=0)[1:], 0.0, atol=1e-12)
    assert r.mean(axis=0)[0] > 0.9


def test_row0_is_up_and_col0_is_left():
    """The convention the bench tool prints and the node's TF assumes."""
    r = ray_table(N).reshape(N, N, 3)
    assert r[0, 0, 2] > 0 and r[-1, 0, 2] < 0        # row 0 up
    assert r[0, 0, 1] > 0 and r[0, -1, 1] < 0        # col 0 left
    assert np.all(np.diff(r[:, 0, 2]) < 0)           # monotonic top->bottom


def test_edge_zone_sits_at_half_the_fov():
    r = ray_table(N).reshape(N, N, 3)
    step = FOV_DEG / N
    expect = FOV_DEG / 2 - step / 2                  # centre of the outermost zone
    assert math.degrees(math.asin(r[0, N // 2, 2])) == pytest.approx(expect, abs=1e-6)


def test_four_by_four_is_supported_and_wider_per_zone():
    assert ray_table(4).shape == (16, 3)
    with pytest.raises(ValueError):
        ray_table(5)


# ------------------------------------------------------------ status filter

@pytest.mark.parametrize('status,kept', [
    (5, True), (9, True),
    (0, False), (4, False), (6, False), (10, False), (12, False), (255, False),
])
def test_only_valid_status_survives(status, kept):
    """Status 6/10/12 carry a plausible-looking distance. Letting them through
    is how a costmap acquires obstacles that were never there."""
    assert valid_mask(*frame(status=status), N).all() == kept


def test_zero_mm_is_rejected_even_with_a_valid_status():
    d, s = frame()
    d[3] = 0.0
    assert not valid_mask(d, s, N)[3]


def test_range_bounds_are_exclusive_of_the_far_field():
    d, s = frame(dist_mm=DEFAULT_MAX_RANGE_M * 1000 + 1)
    assert not valid_mask(d, s, N).any()
    d, s = frame(dist_mm=5.0)                        # inside min_range
    assert not valid_mask(d, s, N).any()


def test_sigma_gate_is_opt_in():
    d, s = frame()
    sig = np.full(N2, 40.0)
    assert valid_mask(d, s, N, sigma_mm=sig).all()             # no threshold -> ignored
    assert not valid_mask(d, s, N, sigma_mm=sig, max_sigma_mm=20.0).any()


# ------------------------------------------------------------- zone ordering

def test_reorder_identity_is_a_noop():
    a = np.arange(N2)
    assert np.array_equal(reorder(a, N), a)


def test_flips_and_transpose_match_the_documented_order():
    a = np.arange(N2).reshape(N, N)
    assert np.array_equal(reorder(a.ravel(), N, flip_h=True), a[:, ::-1].ravel())
    assert np.array_equal(reorder(a.ravel(), N, flip_v=True), a[::-1, :].ravel())
    # transpose first, then flip_v, then flip_h
    expect = a.T[::-1, :][:, ::-1]
    got = reorder(a.ravel(), N, transpose=True, flip_v=True, flip_h=True)
    assert np.array_equal(got, expect.ravel())


def test_reordering_moves_the_point_it_should():
    """A single near return in one corner must land in the opposite corner
    when the frame is flipped -- this is the check the bench tool's hand-in-a-
    corner procedure automates."""
    d, s = frame(dist_mm=2000.0)
    d[0] = 300.0                                     # ULD zone 0
    pts_a, _ = frame_to_points(d, s, N)
    pts_b, _ = frame_to_points(d, s, N, flip_h=True)
    near_a = pts_a[np.argmin(np.linalg.norm(pts_a, axis=1))]
    near_b = pts_b[np.argmin(np.linalg.norm(pts_b, axis=1))]
    assert near_a[1] == pytest.approx(-near_b[1], abs=1e-6)   # y mirrored
    assert near_a[2] == pytest.approx(near_b[2], abs=1e-6)    # z untouched


# ------------------------------------------------------------------- points

def test_points_are_radial_not_cartesian_z():
    """The VL53L5CX reports distance ALONG the zone axis, so every point sits
    at exactly that range from the origin. A depth-camera-style z-conversion
    here would shorten the edge zones."""
    pts, keep = frame_to_points(*frame(dist_mm=1500.0), resolution=N)
    assert keep.all() and len(pts) == N2
    assert np.allclose(np.linalg.norm(pts, axis=1), 1.5, atol=1e-5)


def test_points_are_float32_and_sensor_frame():
    pts, _ = frame_to_points(*frame(), resolution=N)
    assert pts.dtype == np.float32
    assert np.all(pts[:, 0] > 0)                     # everything ahead of the sensor


def test_all_invalid_frame_gives_an_empty_cloud_not_a_crash():
    """Pointed at open space every zone times out. This must be an empty
    cloud, and callers must read it as 'measured nothing', not 'nothing there'."""
    pts, keep = frame_to_points(*frame(status=255), resolution=N)
    assert pts.shape == (0, 3)
    assert not keep.any()


def test_partial_frame_keeps_only_the_valid_zones():
    d, s = frame()
    s[:10] = 255
    pts, keep = frame_to_points(d, s, N)
    assert len(pts) == N2 - 10 == int(keep.sum())


# --------------------------------------------------------------- mount maths

def test_floor_coverage_matches_hand_geometry():
    rows, ranges = floor_coverage(0.055, 25.0, N)
    assert len(rows) == N                            # every row is below horizon
    step = FOV_DEG / N
    down = 25.0 - (N / 2 - 0.5 - rows[0]) * step
    assert ranges[0] == pytest.approx(0.055 / math.tan(math.radians(down)))
    assert np.all(np.diff(ranges) < 0)               # rows sweep inward


def test_steeper_tilt_caps_the_far_field():
    """The reason 25 deg was rejected for a 55 mm mount: it puts every row
    inside 0.6 m, so the sensor cannot see far enough to stop for anything."""
    _, shallow = floor_coverage(0.055, 15.0, N)
    _, steep = floor_coverage(0.055, 25.0, N)
    assert shallow.max() > 3.0
    assert steep.max() < 0.7


def test_rows_above_the_horizon_are_omitted():
    rows, _ = floor_coverage(0.055, 0.0, N)
    assert len(rows) == N // 2                       # exactly half point downward


# ------------------------------------------------------------------- driver

def test_sim_driver_produces_a_recognisable_plane():
    from drivers_x3 import VL53L5CXArray
    tof = VL53L5CXArray(sim_mode=True)
    assert tof.available and tof.data_ready()
    pts = tof.read_points()
    assert pts is not None and len(pts) == N2
    # Rotate the sim's own mount back out: the floor rows must land at z ~ 0.
    pitch = math.radians(15.0)
    z = -pts[:, 0] * math.sin(pitch) + pts[:, 2] * math.cos(pitch) + 0.055
    floor = z[np.abs(z) < 0.02]
    assert len(floor) > N2 // 3


def test_absent_sensor_returns_none_not_empty():
    """None (no sensor) and (0,3) (nothing in range) must stay distinct."""
    from drivers_x3 import VL53L5CXArray
    tof = VL53L5CXArray(sim_mode=False, i2c_bus=99)
    assert not tof.available
    assert tof.read_frame() is None and tof.read_points() is None
    assert not tof.data_ready()


def test_ranging_freq_is_clamped_loudly_per_resolution():
    from drivers_x3 import VL53L5CXArray
    assert VL53L5CXArray(sim_mode=True, resolution=8, ranging_freq_hz=60).ranging_freq_hz == 15
    assert VL53L5CXArray(sim_mode=True, resolution=4, ranging_freq_hz=60).ranging_freq_hz == 60
    with pytest.raises(ValueError):
        VL53L5CXArray(sim_mode=True, resolution=6)


# --------------------------------------------------------------- ROS message

def test_cloud_msg_roundtrips_and_allows_zero_points():
    pytest.importorskip('rclpy')
    from builtin_interfaces.msg import Time
    from yahboomcar_bringup.tof_node import cloud_msg
    stamp = Time()
    pts = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
    m = cloud_msg(pts, 'tof_link', stamp)
    assert m.width == 2 and m.point_step == 12 and m.row_step == 24
    assert np.array_equal(np.frombuffer(m.data, '<f4').reshape(-1, 3), pts)
    empty = cloud_msg(np.zeros((0, 3), np.float32), 'tof_link', stamp)
    assert empty.width == 0 and len(empty.data) == 0


def test_mount_tf_pitches_the_x_axis_downward():
    """Sign check on the static TF: a positive pitch_deg must tip the sensor's
    +x below the horizon, or every point lands above the floor instead of on it."""
    rclpy = pytest.importorskip('rclpy')
    from yahboomcar_bringup.tof_node import TofNode
    rclpy.init(args=['--ros-args', '-p', 'sim:=true', '-p', 'pitch_deg:=20.0'])
    try:
        node = TofNode()
        q = node._mount_tf().transform.rotation
        # Rotate +x by the quaternion; z of the result must be negative.
        x, y, z, w = q.x, q.y, q.z, q.w
        fwd_z = 2 * (x * z - w * y)
        assert fwd_z < 0
        assert math.degrees(math.asin(-fwd_z)) == pytest.approx(20.0, abs=1e-6)
        node.sensor.cleanup()
        node.destroy_node()
    finally:
        rclpy.shutdown()
