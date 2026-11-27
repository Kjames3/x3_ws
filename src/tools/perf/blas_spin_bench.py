"""Measure process CPU and small NumPy call latency with isolated BLAS settings."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def run_periodic(operation, hz, seconds):
    period = 1 / hz
    deadline = time.perf_counter()
    end = deadline + seconds
    latencies = []
    while deadline < end:
        time.sleep(max(0, deadline - time.perf_counter()))
        start = time.perf_counter()
        if start >= end:
            break
        operation()
        latencies.append((time.perf_counter() - start) * 1e6)
        deadline = max(deadline + period, start)
    return latencies


def measure(seconds, activation_size):
    import numpy as np

    rng = np.random.default_rng(42)
    mat = rng.normal(size=(3, 3))
    xyz = rng.normal(size=(200, 3))
    covariance = mat.T @ mat + np.eye(3)
    jobs = [
        ('dot 3x3', lambda: np.dot(mat, mat), 100),
        ('matmul 200x3', lambda: xyz @ mat, 50),
        ('eigh 3x3', lambda: np.linalg.eigh(covariance), 30),
        ('solve 3x3', lambda: np.linalg.solve(covariance, mat[0]), 10),
    ]
    if activation_size:
        large = rng.normal(size=(activation_size, activation_size))
        # Positive control to exercise the BLAS pool on builds that keep the
        # small operations above single-threaded.
        jobs.append((f'matmul {activation_size}x{activation_size}', lambda: large @ large, 10))
    for _, operation, _ in jobs:
        operation()

    proc_tasks = Path('/proc/self/task')
    native_threads = len(list(proc_tasks.iterdir())) if proc_tasks.is_dir() else None
    wall_start, cpu_start = time.perf_counter(), time.process_time()
    with ThreadPoolExecutor(max_workers=len(jobs)) as executor:
        futures = [executor.submit(run_periodic, operation, hz, seconds)
                   for _, operation, hz in jobs]
        samples = [future.result() for future in futures]
    cpu = time.process_time() - cpu_start
    wall = time.perf_counter() - wall_start
    return {
        'wall_s': wall,
        'cpu_s': cpu,
        'threads_after_warmup': native_threads,
        'calls': {
            name: [len(latencies), *np.percentile(latencies, [50, 95]).tolist()]
            for (name, _, _), latencies in zip(jobs, samples)
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--activation-size', type=int, default=0,
                        help='add a square matrix multiply of this size at 10 Hz')
    parser.add_argument('--child', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.seconds < 0.2 or args.activation_size < 0:
        parser.error('use at least 0.2 seconds and a nonnegative activation size')
    if args.child:
        print(json.dumps(measure(args.seconds, args.activation_size)))
        return

    # Clear thread limits that could mask the unset-vs-one comparison.
    env = os.environ.copy()
    for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'GOTO_NUM_THREADS',
                'OPENBLAS_DEFAULT_NUM_THREADS'):
        env.pop(key, None)
    command = [sys.executable, __file__, '--child', '--seconds', str(args.seconds),
               '--activation-size', str(args.activation_size)]
    for label, child_env in [('unset', env), ('1', dict(env, OPENBLAS_NUM_THREADS='1'))]:
        stats = json.loads(subprocess.check_output(command, env=child_env, text=True))
        cpu, wall = stats['cpu_s'], stats['wall_s']
        print(f'OPENBLAS_NUM_THREADS={label} ({stats["threads_after_warmup"]} process threads'
              f' after warmup): {cpu:.3f} CPU s / {wall:.3f} wall s = {100 * cpu / wall:.2f}% CPU')
        for name, (count, p50, p95) in stats['calls'].items():
            print(f'  {name:22s} {count:5d} calls   p50 {p50:8.2f} us   p95 {p95:8.2f} us')


if __name__ == '__main__':
    main()
