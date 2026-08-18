"""Directly time the /proc scan that _is_standalone_test_running() performs.

Independent confirmation of the py-spy sampling result: measures the wall time of
one psutil.process_iter(['cmdline']) walk, which motion_loop triggers once a second
on the asyncio event loop.
"""
import time

import psutil


def scan():
    for proc in psutil.process_iter(['cmdline']):
        cmd = proc.info.get('cmdline')
        if cmd:
            cmd_str = ' '.join(cmd)
            if 'ab_comparison_test.py' in cmd_str or 'point_to_point_test.py' in cmd_str:
                if 'server_x3.py' not in cmd_str:
                    return True
    return False


times = []
for i in range(7):
    t0 = time.perf_counter()
    scan()
    times.append((time.perf_counter() - t0) * 1000.0)
    time.sleep(0.5)

times_sorted = sorted(times)
print("process count:", len(psutil.pids()))
print("scan times (ms):", " ".join(f"{t:.1f}" for t in times))
print(f"median {times_sorted[len(times)//2]:.1f} ms   "
      f"min {times_sorted[0]:.1f} ms   max {times_sorted[-1]:.1f} ms")
