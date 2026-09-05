import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_deskew import deskew, interpolate_pitch, rotate, slerp


def quat(axis, angle):
    q = np.zeros(4)
    q[axis] = np.sin(angle / 2)
    q[3] = np.cos(angle / 2)
    return q


def test_stationary_geometry_preserves_laser_points():
    points = np.array([[1., 0, 0], [0, 2., 0], [-3., 1, 0]])
    mount = (np.array([1., 2., .165]), quat(2, .6))
    pivot = np.array([-.0125, 0, .153])
    laser = (np.array([0., 0., .022]), quat(2, np.pi))
    pitch = .4
    # Independent homogeneous matrix composition, including the moving offset.
    def matrix(pose):
        t, q = pose
        result = np.eye(4)
        result[:3, :3] = rotate(np.eye(3), q).T
        result[:3, 3] = t
        return result
    total = matrix(mount) @ matrix((pivot, quat(1, pitch))) @ matrix(laser)
    world = (total @ np.c_[points, np.ones(3)].T).T[:, :3]
    # Use identity reference to check the complete forward chain.
    actual = deskew(points, np.linspace(0, 1, 3), np.full(3, pitch),
                    mount, mount, pivot, laser, (np.zeros(3), quat(2, 0)))
    np.testing.assert_allclose(actual, world, atol=3e-7)


def test_moving_sensor_reconstructs_fixed_wall():
    n = 31
    f = np.linspace(0, 1, n)
    pitch = -.3 + .6 * f
    yaw = .2 * f
    origins = np.c_[.1 * f, -.04 * f, np.full(n, .34)]
    # Synthetic ray along sensor X, intersecting world wall X=3.
    directions = np.c_[np.cos(yaw) * np.cos(pitch),
                        np.sin(yaw) * np.cos(pitch), -np.sin(pitch)]
    ranges = (3 - origins[:, 0]) / directions[:, 0]
    points = np.c_[ranges, np.zeros((n, 2))]
    actual = deskew(points, f, pitch,
                    (origins[0], quat(2, 0)), (origins[-1], quat(2, .2)),
                    np.zeros(3), (np.zeros(3), quat(2, 0)),
                    (np.zeros(3), quat(2, 0)))
    np.testing.assert_allclose(actual, origins + directions * ranges[:, None], atol=5e-7)
    np.testing.assert_allclose(actual[:, 0], 3, atol=5e-7)


def test_reference_origin_and_rotation():
    points = np.array([[2., 1, 0]])
    pose = (np.array([4., -2., .34]), quat(2, .8))
    actual = deskew(points, np.array([0.]), np.array([0.]), pose, pose,
                    np.zeros(3), (np.zeros(3), quat(2, 0)), pose)
    np.testing.assert_allclose(actual, points, atol=1e-6)


def test_pitch_interpolation_and_slerp_sign():
    samples = [(0., -.4, True), (.1, .4, True)]
    np.testing.assert_allclose(interpolate_pitch(samples, np.array([0, .05, .1]), .12),
                               [-.4, 0, .4], atol=1e-15)
    q = quat(2, .7)
    np.testing.assert_allclose(slerp(q, -q, [0, .5, 1]), np.tile(q, (3, 1)))


@pytest.mark.parametrize('samples,times', [
    ([(0., 0, True)], [0, .1]),
    ([(0., 0, True), (.2, 0, True)], [0, .2]),
    ([(0., 0, True), (.1, 0, True)], [-.01, .05]),
    ([(0., 0, True), (.1, 0, True)], [0, .11]),
    ([(0., 0, True), (.1, 0, False)], [0, .1]),
    ([(.1, 0, True), (0., 0, True)], [0, .1]),
    ([(0., float('nan'), True), (.1, 0, True)], [0, .1]),
])
def test_invalid_history_withholds(samples, times):
    with pytest.raises(ValueError):
        interpolate_pitch(samples, np.array(times), .12)
