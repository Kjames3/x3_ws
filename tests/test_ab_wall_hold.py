"""Wall early-stop in ab_comparison_test: hold before ending a leg, and a
person seen close ahead is not a wall."""
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('websockets')

import ab_comparison_test as ab  # noqa: E402


class _Log:
    def info(self, *a, **k): pass
    def warn(self, *a, **k): pass


def _node():
    n = ab.ABComparisonTest.__new__(ab.ABComparisonTest)
    n._front_is_continuous_wall = True
    n._min_forward_lidar = 0.25
    n._wall_since = None
    n.is_paused = False
    n.stopped = 0
    n._stop_robot = lambda: setattr(n, 'stopped', n.stopped + 1)
    n.get_logger = lambda: _Log()
    return n


def test_wall_holds_then_stops():
    n = _node()
    assert n._wall_stop_check(100.0) == 'hold'
    assert n.is_paused and n.stopped == 1
    assert n._wall_stop_check(100.0 + ab.WALL_HOLD_S - 0.1) == 'hold'
    assert n._wall_stop_check(100.0 + ab.WALL_HOLD_S + 0.1) == 'stop'


def test_wall_that_moves_away_resets_the_hold():
    n = _node()
    assert n._wall_stop_check(100.0) == 'hold'
    n._front_is_continuous_wall = False          # the "wall" walked off
    assert n._wall_stop_check(101.0) is None
    n._front_is_continuous_wall = True
    assert n._wall_stop_check(103.0) == 'hold'   # timer restarted, no stop yet
    assert n._wall_stop_check(103.0 + ab.WALL_HOLD_S - 0.1) == 'hold'


def test_far_wall_is_ignored():
    n = _node()
    n._min_forward_lidar = ab.WALL_STOP_DIST + 0.05
    assert n._wall_stop_check(100.0) is None


def test_turn_minimum_is_inside_the_validated_wheel_range():
    # yaw_wheel_separation 0.215 m; the wheel loop is validated from 0.05 m/s.
    assert ab.MIN_ANGULAR_SPEED * 0.215 >= 0.05
