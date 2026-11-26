"""Measure server perf flag paths on Linux without starting ROS2 or hardware."""
import ast
import asyncio
import logging
import os
from pathlib import Path
from statistics import median
import subprocess
import tempfile
import threading
import time
from timeit import repeat
from types import SimpleNamespace


def load_paths():
    # Importing the entire server parses CLI arguments and pulls in robot dependencies.
    source = Path(__file__).resolve().parents[2] / 'server_x3.py'
    names = {
        'OLED_GUARD', '_get_ssid', '_get_ip', 'oled_loop',
        'PSUTIL_SCAN_THREAD', '_STANDALONE_TESTS', '_STANDALONE_TEST_SCAN_INTERVAL',
        '_standalone_test_cache', '_standalone_scan_busy', '_scan_standalone_tests',
        '_is_standalone_test_running', 'CBF_FLAG_CACHE', 'CBF_OFF_FILE', '_cbf_off_cache',
        '_cbf_off',
    }
    picked = []
    for node in ast.parse(source.read_text()).body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in names:
            picked.append(node)
        elif isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', None) in names:
            picked.append(node)
    server = dict(os=os, subprocess=subprocess,
                  threading=threading, time=time, logger=logging.getLogger(__name__),
                  connected_clients=set(), WS_PORT=8081)
    exec(compile(ast.Module(body=picked, type_ignores=[]), str(source), 'exec'), server)
    return server


async def oled_turn(server, enabled, has_device):
    async def end_refresh(_):
        server['_shutting_down'] = True

    server.update(OLED_GUARD=enabled, _shutting_down=False,
                  oled=SimpleNamespace(has_device=has_device, show=lambda lines: None),
                  asyncio=SimpleNamespace(sleep=end_refresh, to_thread=asyncio.to_thread))
    elapsed = asyncio.get_running_loop().create_future()
    start = time.perf_counter_ns()
    refresh = asyncio.create_task(server['oled_loop']())
    # Queued immediately after the OLED task. Runs once it yields or finishes.
    asyncio.get_running_loop().call_soon(lambda: elapsed.set_result(time.perf_counter_ns() - start))
    await refresh
    return elapsed.result()


async def scan_call(server, enabled):
    server.update(PSUTIL_SCAN_THREAD=enabled, _standalone_test_cache=(False, 0.0))
    start = time.perf_counter_ns()
    server['_is_standalone_test_running']()
    elapsed = time.perf_counter_ns() - start
    # Run the actual legacy psutil job, but wait for its completion outside the timing.
    while server['_standalone_scan_busy']:
        await asyncio.sleep(0.001)
    return elapsed


async def main():
    import psutil

    server = load_paths()
    await asyncio.to_thread(lambda: None)  # Warm the executor for the connected OLED path.
    print(f'{len(psutil.pids())} visible processes, 100 samples per event-loop path')
    for name, operation in (
        ('OLED unplugged, first loop turn', lambda enabled: oled_turn(server, enabled, False)),
        ('OLED connected, first loop turn', lambda enabled: oled_turn(server, enabled, True)),
        ('Standalone check when due', lambda enabled: scan_call(server, enabled)),
    ):
        samples = {False: [], True: []}
        for index in range(101):
            for enabled in (bool(index % 2), not bool(index % 2)):
                duration = await operation(enabled)
                if index:
                    samples[enabled].append(duration)
        print(f'{name}: {median(samples[False]) / 1e3:.2f} -> '
              f'{median(samples[True]) / 1e3:.2f} µs median (default -> enabled)')

    with tempfile.TemporaryDirectory() as directory:
        server['CBF_OFF_FILE'] = os.path.join(directory, 'cbf_off')
        timings = []
        for enabled in (False, True):
            server['CBF_FLAG_CACHE'] = enabled
            server['_cbf_off']()
            calls = 100_000
            timings.append(median(repeat(server['_cbf_off'], number=calls, repeat=5)) / calls)
    print(f'CBF override check, warm cache: {timings[0] * 1e6:.3f} -> '
          f'{timings[1] * 1e6:.3f} µs per call (default -> enabled)')


if __name__ == '__main__':
    asyncio.run(main())
