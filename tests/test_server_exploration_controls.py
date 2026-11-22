"""Run the server's motion and exploration inputs without ROS or hardware."""
import ast
import asyncio
import logging
import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.fixture
def server():
    source = Path(__file__).parents[1] / "src" / "server_x3.py"
    tree = ast.parse(source.read_text())
    names = {
        "_motion_inhibited", "_gate_route", "_enqueue_motion", "_cancel_test_prep",
        "_stop_manual_motion", "_publish_motion_stop", "_stop_all_motion",
        "_resume_motion", "_start_frontier_exploration",
    }
    methods = {"_map_cb", "get_occupancy_grid", "get_exploration_pose", "_gate_cmd_vel_cb"}
    bridge = next(node for node in tree.body
                  if isinstance(node, ast.ClassDef) and node.name == "ROS2Bridge")
    picked = [node for node in tree.body
              if isinstance(node, ast.FunctionDef) and node.name in names]
    picked += [node for node in bridge.body
               if isinstance(node, ast.FunctionDef) and node.name in methods]
    assert {node.name for node in picked} == names | methods
    env = dict(asyncio=asyncio, time=NS(monotonic=lambda: 42.0), np=np, math=math,
               logger=logging.getLogger(__name__), motors_enabled=True, nav2_client=None,
               ros_bridge=None, ros_board=None, frontier_explorer=None,
               _ab_test_proc=None, _p2p_proc=None, _test_prep=None,
               _is_standalone_test_running=lambda: False)
    exec(compile(ast.Module(body=picked, type_ignores=[]), str(source), "exec"), env)
    return env


def test_operator_stop_latches_before_cancel_and_requires_explicit_resume(server):
    events = []
    goal = NS(busy=True, cancelling=False)
    proc = NS(running=True)
    server.update(
        motion_queue=asyncio.Queue(),
        is_auto_driving=True,
        ros_bridge=NS(stop=lambda: events.append(("zero", server["motors_enabled"]))),
        frontier_explorer=NS(stop=lambda: events.append("explorer")),
        nav2_client=NS(cancel=lambda: events.append("nav"),
                       is_cancelling=lambda: goal.cancelling, is_busy=lambda: goal.busy),
        _ab_test_proc=NS(poll=lambda: None if proc.running else 0,
                         terminate=lambda: events.append("test")),
        _test_prep={"kind": "p2p"},
    )
    queue = server["motion_queue"]
    queue.put_nowait((0.4, 0.0, 0.0, False))
    server["_stop_all_motion"]()
    assert events == [("zero", False), "explorer", "nav", "test"]
    assert not server["is_auto_driving"]
    assert server["_test_prep"] is None
    server["_enqueue_motion"](0.4, 0.0, 0.0)
    assert queue.get_nowait() == (0.0, 0.0, 0.0, True)
    assert queue.empty()
    assert not server["_resume_motion"]()
    goal.busy = False
    assert not server["_resume_motion"]()
    proc.running = False
    assert server["_motion_inhibited"]()
    assert server["_resume_motion"]()
    assert not server["_motion_inhibited"]()


@pytest.mark.parametrize("enabled,cancelling", [(False, False), (True, True)])
@pytest.mark.parametrize("bench_test", [False, True])
def test_gate_sends_exact_zero_through_inhibit(server, enabled, cancelling, bench_test):
    class Twist:
        def __init__(self, speed=0.0):
            self.linear = NS(x=speed, y=0.0)
            self.angular = NS(z=0.0)

    def filter_motion(*_):
        pytest.fail("Inhibited commands must bypass the CBF")

    output = []
    bridge = NS(_move_lock=threading.Lock(), _cmd_vel_pub=NS(publish=output.append),
                _move_impl=filter_motion)
    server.update(motors_enabled=enabled, nav2_client=NS(is_cancelling=lambda: cancelling),
                  _is_standalone_test_running=lambda: bench_test)
    server["_gate_cmd_vel_cb"](bridge, Twist(0.3))
    assert len(output) == 1
    assert output[0].linear.x == 0.0


def test_explorer_pose_requires_fresh_map_tf(server, monkeypatch):
    class TransformException(Exception):
        pass

    class Time:
        @staticmethod
        def from_msg(msg):
            return msg

    transform = NS(header=NS(stamp=NS(nanoseconds=9_800_000_000)),
                   transform=NS(translation=NS(x=7.0, y=-2.0),
                                rotation=NS(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5))))

    def lookup(frame, child, _):
        assert (frame, child) == ("map", "base_footprint")
        return transform

    def missing_transform(*_):
        raise TransformException()

    monkeypatch.setitem(sys.modules, "rclpy.time", NS(Time=Time))
    monkeypatch.setitem(sys.modules, "tf2_ros", NS(TransformException=TransformException))
    bridge = NS(_node=NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))),
                _explore_tf=NS(lookup_transform=lookup))
    pose = server["get_exploration_pose"]
    assert pose(bridge, "map") == pytest.approx((7.0, -2.0, math.pi / 2))
    assert pose(bridge, "odom") is None
    transform.header.stamp.nanoseconds = 7_000_000_000
    assert pose(bridge, "map") is None
    bridge._explore_tf.lookup_transform = missing_transform
    assert pose(bridge, "map") is None


def test_occupancy_snapshot_captures_metadata_and_immutable_cells(server):
    bridge = NS(_lock=threading.Lock(), map_count=0, _occupancy_grid=None)
    origin = NS(position=NS(x=1.0, y=2.0),
                orientation=NS(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5)))
    message = NS(header=NS(frame_id="map"), data=[-1, 0, 100],
                 info=NS(width=3, height=1, resolution=0.05, origin=origin))
    server["_map_cb"](bridge, message)
    first = server["get_occupancy_grid"](bridge)
    assert first["frame_id"] == "map"
    assert first["origin_yaw"] == pytest.approx(math.pi / 2)
    assert first["received_monotonic"] == 42.0
    assert not first["data"].flags.writeable
    message.data[0] = 100
    server["_map_cb"](bridge, message)
    second = server["get_occupancy_grid"](bridge)
    assert second["sequence"] == first["sequence"] + 1
    assert first["data"].tolist() == [-1, 0, 100]
    assert second["data"].tolist() == [100, 0, 100]
    server["_map_cb"](bridge, message)
    repeated = server["get_occupancy_grid"](bridge)
    assert repeated["sequence"] == second["sequence"]
    assert repeated["data"] is second["data"]
    assert bridge.map_count == 3
    origin.position.x += 1
    server["_map_cb"](bridge, message)
    assert server["get_occupancy_grid"](bridge)["sequence"] > repeated["sequence"]
