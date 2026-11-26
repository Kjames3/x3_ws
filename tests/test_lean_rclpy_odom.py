"""Exercise the bridge's odometry subscriptions without ROS or robot hardware."""

import ast
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.fixture
def odom_bridge(monkeypatch):
    # Follow the server tests' AST approach to avoid importing camera/YOLO dependencies.
    source = Path(__file__).parents[1] / 'src' / 'server_x3.py'
    tree = ast.parse(source.read_text())
    bridge = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == 'ROS2Bridge')
    names = {'_init_odom_subscriptions', '_subscribe_raw_odom', '_check_raw_odom',
             '_lean_fused_odom_cb', '_odom_cb', '_people_cloud_tick'}
    bridge.body = [node for node in bridge.body
                   if isinstance(node, ast.FunctionDef) and node.name in names]
    clock = NS(t=100.0)
    env = {'time': NS(monotonic=lambda: clock.t), 'SIM_MODE': False,
           'PEOPLE_CLOUD_PERIOD_S': 0.1, 'PEOPLE_COSTMAP_ENABLED': False}
    exec(compile(ast.Module(body=[bridge], type_ignores=[]), str(source), 'exec'), env)

    steady = object()
    monkeypatch.setitem(sys.modules, 'rclpy.clock', NS(
        Clock=Mock(return_value=steady), ClockType=NS(STEADY_TIME='steady')))
    subscriptions = {}

    def subscribe(msg_type, topic, callback, depth):
        assert msg_type is object and depth == 10
        assert topic not in subscriptions
        subscriptions[topic] = callback
        return topic

    def destroy(topic):
        del subscriptions[topic]
        return True

    instance = env['ROS2Bridge']()
    instance._lock = threading.Lock()
    instance.data_watchdog = Mock()
    instance._velocity_pose_history = Mock()
    instance._node = NS(
        create_subscription=Mock(side_effect=subscribe),
        destroy_subscription=Mock(side_effect=destroy),
        create_timer=Mock(),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)))
    return instance, clock, subscriptions, steady, env


def odometry(x):
    return NS(header=NS(stamp=NS(sec=1, nanosec=0)),
              pose=NS(pose=NS(position=NS(x=x, y=0.0),
                              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              twist=NS(twist=NS(linear=NS(x=x, y=0.0), angular=NS(z=0.0))))


@pytest.mark.parametrize('sim', [False, True])
def test_lean_starts_raw_only_after_one_second_without_fused_odom(odom_bridge, sim):
    bridge, clock, subscriptions, steady, env = odom_bridge
    env['SIM_MODE'] = sim
    bridge._init_odom_subscriptions(object, lean=True)
    if sim:
        period, tick = bridge._node.create_timer.call_args.args
        assert period == 0.1 and bridge._node.create_timer.call_args.kwargs['clock'] is steady
    else:
        bridge._node.create_timer.assert_not_called()
        tick = bridge._people_cloud_tick
    assert set(subscriptions) == {'/odom'}
    clock.t += 1.0
    tick()
    assert '/odom_raw' not in subscriptions
    clock.t += 0.01
    tick()
    tick()
    assert set(subscriptions) == {'/odom', '/odom_raw'}
    subscriptions['/odom_raw'](odometry(2.0))
    assert bridge._pose_m['x'] == bridge._twist['vx'] == 2.0
    assert bridge._odom_stamp == clock.t
    bridge.data_watchdog.beat.assert_not_called()
    bridge._velocity_pose_history.add.assert_not_called()


def test_lean_recovers_to_fused_and_reopens_on_next_outage(odom_bridge):
    bridge, clock, subscriptions, _, _ = odom_bridge
    bridge._init_odom_subscriptions(object, lean=True)
    tick = bridge._people_cloud_tick
    clock.t = 100.75
    subscriptions['/odom'](odometry(1.0))
    clock.t = 101.1
    tick()
    assert '/odom_raw' not in subscriptions
    clock.t = 101.8
    tick()
    raw = subscriptions['/odom_raw']
    raw(odometry(2.0))
    assert bridge._fused_odom_stamp == 100.75

    subscriptions['/odom'](odometry(3.0))
    bridge._node.destroy_subscription.assert_called_once_with('/odom_raw')
    raw(odometry(2.0))  # A previously queued raw sample cannot overwrite recovery.
    assert bridge._pose_m['x'] == bridge._twist['vx'] == 3.0
    assert '/odom_raw' not in subscriptions
    assert bridge.data_watchdog.beat.call_count == 2
    assert bridge._velocity_pose_history.add.call_count == 2
    clock.t = 102.9
    tick()
    assert '/odom_raw' in subscriptions


def test_default_keeps_raw_subscription_and_half_second_fallback(odom_bridge):
    bridge, clock, subscriptions, _, _ = odom_bridge
    bridge._init_odom_subscriptions(object, lean=False)
    bridge._node.create_timer.assert_not_called()
    raw = subscriptions['/odom_raw']
    raw(odometry(2.0))
    assert bridge._pose_m['x'] == 2.0
    subscriptions['/odom'](odometry(1.0))
    clock.t += 0.49
    raw(odometry(2.0))
    assert bridge._pose_m['x'] == 1.0
    clock.t = 100.5
    raw(odometry(2.0))
    assert bridge._pose_m['x'] == 2.0
    bridge._node.destroy_subscription.assert_not_called()
