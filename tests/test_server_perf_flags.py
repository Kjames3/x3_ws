"""Exercise the opt-in server paths without importing ROS2 or YOLO."""
import ast
import asyncio
import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drivers_x3 import OLEDDisplay


SERVER_PATH = Path(__file__).resolve().parents[1] / "src" / "server_x3.py"
FLAGS = ("CBF_FLAG_CACHE", "OLED_GUARD", "PSUTIL_SCAN_THREAD")
NAMES = {
    *FLAGS, "CBF_OFF_FILE", "_cbf_off_cache", "_cbf_off", "_get_ip", "_get_ssid",
    "oled_loop", "_standalone_test_cache", "_STANDALONE_TEST_SCAN_INTERVAL",
    "_STANDALONE_TESTS", "_standalone_scan_busy", "_scan_standalone_tests",
    "_standalone_test_scan_loop", "_is_standalone_test_running", "main",
}


def load_server():
    tree = ast.parse(SERVER_PATH.read_text())
    selected = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            name = node.name
        elif isinstance(node, ast.Assign):
            name = getattr(node.targets[0], "id", None)
        else:
            continue
        if name in NAMES:
            selected[name] = node
    assert selected.keys() == NAMES
    server = dict(asyncio=asyncio, os=os, subprocess=subprocess, threading=threading,
                  time=time, _shutting_down=False, logger=logging.getLogger(__name__))
    exec(compile(ast.Module(body=list(selected.values()), type_ignores=[]),
                 str(SERVER_PATH), "exec"), server)
    return server


@pytest.fixture(params=[False, True], ids=["default", "enabled"])
def server(request, monkeypatch):
    for flag in FLAGS:
        monkeypatch.delenv(f"X3_{flag}", raising=False)
        if request.param:
            monkeypatch.setenv(f"X3_{flag}", "1")
    server = load_server()
    assert all(server[flag] is request.param for flag in FLAGS)
    return server


@pytest.mark.parametrize("has_device", [False, True], ids=["unplugged", "connected"])
def test_oled_network_subprocesses(server, has_device):
    main_thread = threading.get_ident()
    worker_calls = []

    def run(command, **kwargs):
        worker_calls.append((command, threading.get_ident()))
        return SimpleNamespace(stdout="Lab WiFi\n" if command[0] == "iwgetid" else "10.0.0.2\n")

    async def stop_after_refresh(interval):
        assert interval == 5
        server["_shutting_down"] = True

    oled = OLEDDisplay(sim_mode=True)
    if has_device:
        oled._device = object()
    oled.show = Mock()
    server.update(asyncio=SimpleNamespace(sleep=stop_after_refresh, to_thread=asyncio.to_thread),
                  subprocess=SimpleNamespace(run=run), oled=oled, connected_clients={1},
                  WS_PORT=8081)
    asyncio.run(server["oled_loop"]())

    if server["OLED_GUARD"] and not has_device:
        assert worker_calls == []
        oled.show.assert_not_called()
    else:
        assert [command for command, _ in worker_calls] == [["iwgetid", "--raw"], ["hostname", "-I"]]
        assert all((ident != main_thread) == server["OLED_GUARD"] for _, ident in worker_calls)
        oled.show.assert_called_once_with(["WiFi:Lab WiFi", "IP:10.0.0.2", "WS:8081 C:1"])


def test_standalone_scanner_publishes_same_results(server, monkeypatch):
    running = server["_is_standalone_test_running"]
    enabled = server["PSUTIL_SCAN_THREAD"]
    now = 100.0
    test_running = True
    def processes(attrs):
        assert attrs == ["cmdline"]
        cmdlines = [None, [], ["python3", "server_x3.py", "point_to_point_test.py"]]
        if test_running:
            cmdlines.append(["python3", "/tmp/tools/point_to_point_test.py"])
        return [SimpleNamespace(info={"cmdline": cmdline}) for cmdline in cmdlines]

    process_iter = Mock(side_effect=processes)
    monkeypatch.setitem(sys.modules, "psutil", SimpleNamespace(process_iter=process_iter))

    def wait(interval):
        nonlocal now, test_running
        assert interval == 1.0
        now += interval
        if test_running:
            assert running()
            test_running = False
        else:
            assert not running()
            server["_shutting_down"] = True

    def start_worker(*, target, name, daemon):
        assert name == "standalone_scan"
        assert daemon
        return SimpleNamespace(start=target)

    thread = Mock(side_effect=start_worker)
    server.update(time=SimpleNamespace(monotonic=lambda: now, sleep=wait),
                  threading=SimpleNamespace(Thread=thread),
                  _start_http_server=Mock(side_effect=RuntimeError("stop before hardware")))

    assert not running()
    with pytest.raises(RuntimeError, match="stop before hardware"):
        asyncio.run(server["main"]())
    if enabled:
        thread.assert_called_once_with(target=server["_standalone_test_scan_loop"],
                                       name="standalone_scan", daemon=True)
    else:
        assert running()
        now += 1.0
        test_running = False
        assert running()  # The refresh returns the preceding snapshot.
        assert not running()
        assert thread.call_count == 2
        assert all(call.kwargs["target"] == server["_scan_standalone_tests"]
                   for call in thread.call_args_list)
    assert process_iter.call_count == 2


def test_cbf_override_creation_and_removal(server):
    enabled = server["CBF_FLAG_CACHE"]
    now = 0.0
    exists = Mock(return_value=False)
    server.update(time=SimpleNamespace(monotonic=lambda: now),
                  os=SimpleNamespace(path=SimpleNamespace(exists=exists)))
    cbf_off = server["_cbf_off"]

    assert not cbf_off()
    exists.return_value = True
    now = 0.5
    assert cbf_off() is not enabled
    now = 1.0
    assert cbf_off()
    exists.return_value = False
    now = 1.5
    assert cbf_off() is enabled
    now = 2.0
    assert not cbf_off()
    assert exists.call_count == (3 if enabled else 5)
    assert all(call.args == (server["CBF_OFF_FILE"],) for call in exists.call_args_list)
