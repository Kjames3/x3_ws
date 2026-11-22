"""Server-managed SLAM and Nav2 process selection, without starting ROS."""
import ast
import logging
from pathlib import Path
import signal
import threading
from types import SimpleNamespace

import pytest


@pytest.fixture
def launch():
    path = Path(__file__).resolve().parents[1] / "src" / "server_x3.py"
    names = {
        "_slam_running", "_launch_nav2_stack", "_launch_nav2_seeded",
        "_start_gui_slam", "_launch_gui_nav2", "_stop_gui_slam",
    }
    functions = [node for node in ast.parse(path.read_text()).body
                 if isinstance(node, ast.FunctionDef) and node.name in names]
    calls = []
    state = {"nav2_running": False, "external_slam": False}

    def launch_nav2(**kwargs):
        if state["nav2_running"]:
            return False
        calls.append(("nav2", kwargs))
        state.update(nav2_running=True, external_slam=kwargs["external_slam"])
        return True

    def stop_nav2():
        calls.append(("stop_nav2",))
        state.update(nav2_running=False, external_slam=False)

    def launch_slam(**kwargs):
        calls.append(("slam", kwargs))
        proc = SimpleNamespace(pid=1, exit_code=None)
        proc.poll = lambda: proc.exit_code

        def wait(timeout):
            proc.exit_code = 0

        proc.wait = wait
        return proc

    def killpg(pid, sig):
        calls.append(("signal_slam", sig))

    namespace = {
        "_slam_proc": None,
        "_slam_stopping": False,
        "_nav_launch_lock": threading.RLock(),
        "_launch_slam": launch_slam,
        "_resolve_map_path": lambda name: f"/maps/{name}" if name else None,
        "nav2_client": SimpleNamespace(
            launch_nav2=launch_nav2, stop_nav2=stop_nav2, get_status=lambda: state,
        ),
        "ros_bridge": None,
        "os": SimpleNamespace(killpg=killpg),
        "signal": signal,
        "logger": logging.getLogger(__name__),
    }
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
    return SimpleNamespace(
        start_slam=namespace["_start_gui_slam"],
        stop_slam=namespace["_stop_gui_slam"],
        slam_running=namespace["_slam_running"],
        launch_nav2=namespace["_launch_gui_nav2"],
        calls=calls,
    )


@pytest.mark.parametrize("start_slam_first", [True, False])
def test_mapping_uses_one_standalone_mapper_and_navigation_only(launch, start_slam_first):
    if start_slam_first:
        assert launch.start_slam(True)[0]

    assert launch.launch_nav2(True, "old-map.yaml", not start_slam_first)[0]
    assert launch.slam_running()
    assert launch.start_slam(True)[0]
    assert launch.calls == [
        ("slam", {"use_sim_time": True}),
        ("nav2", {"use_sim_time": True, "map_path": None, "external_slam": True}),
    ]

    launch.stop_slam()
    assert launch.calls[-2:] == [("stop_nav2",), ("signal_slam", signal.SIGINT)]
    assert not launch.slam_running()


def test_saved_map_navigation_refuses_second_map_source(launch):
    assert launch.launch_nav2(True, "saved-map.yaml", False)[0]
    assert not launch.start_slam(True)[0]
    assert not launch.launch_nav2(True, "", True)[0]
    assert not launch.slam_running()
    assert launch.calls == [
        ("nav2", {"use_sim_time": True, "map_path": "/maps/saved-map.yaml", "external_slam": False}),
    ]
