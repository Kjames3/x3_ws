"""Time src/oak_floor_obstacles.py old path vs X3_OAK_FLOOR_FAST=1, per step.

Synthetic front-OAK depth (tilted floor + boxes + speckle + holes, ~10k points
after stride 4), rendered analytically, so this runs on the laptop without ROS.
Each OPENBLAS_NUM_THREADS setting runs in its own subprocess, because OpenBLAS
reads it when numpy is first imported.

    python3 src/tools/perf/oak_floor_bench.py                 # both BLAS settings
    python3 src/tools/perf/oak_floor_bench.py --child         # this process only

Reported per step: median wall time per call (tight loop), and process CPU %
over a 5 Hz loop of the whole step(), which is where OpenBLAS pool spin shows.
`make_scene` / `make_stepper` are also used by tests/test_oak_floor_fast.py.
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import numpy as np  # noqa: E402

import oak_floor_obstacles as ofo  # noqa: E402


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# optical (x right, y down, z forward) -> base (x forward, y left, z up)
_R_BASE_OPT = np.array([[0.0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def make_scene(seed=0, w=640, h=400, pitch_deg=28.0, cam_xyz=(0.20, 0.0, 0.213),
               floor=(0.02, -0.015, 0.03), boxes=None, noise=0.003,
               outlier_frac=0.02, hole_frac=0.05, robot=(1.3, -0.4, 0.6)):
    """Analytic depth image of a tilted floor plane plus axis-aligned boxes.

    floor = (a, b, c): base-frame floor z = a*x + b*y + c.  robot = (x, y, yaw)
    of base_footprint in odom.  Returns a dict with the depth image (float32 m,
    NaN holes), intrinsics tuple, and camera/base -> odom transforms.
    """
    rng = np.random.default_rng(seed)
    if boxes is None:   # (min xyz, max xyz) in base: power strip, low bar, a shoe
        boxes = [((0.75, -0.25, 0.0), (0.82, 0.05, 0.05)),
                 ((1.05, 0.15, 0.0), (1.15, 0.35, 0.09)),
                 ((0.60, 0.30, 0.0), (0.68, 0.38, 0.12))]
    fx = fy = 0.78 * w
    cx, cy = w / 2 - 3.5, h / 2 + 2.25
    R_bc = _rot_y(math.radians(pitch_deg)) @ _R_BASE_OPT
    p0 = np.asarray(cam_xyz, float)
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    d_opt = np.stack(((u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)), -1)
    d = d_opt @ R_bc.T                                 # base-frame ray, optical z = 1
    a, b, c = floor
    den = d[..., 2] - a * d[..., 0] - b * d[..., 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (a * p0[0] + b * p0[1] + c - p0[2]) / den
    t = np.where(t > 0, t, np.inf)
    for lo, hi in boxes:
        lo = np.array(lo, float)
        hi = np.array(hi, float)
        lo[2] += c                                     # boxes sit on the floor
        hi[2] += c
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo - p0) / d
            t2 = (hi - p0) / d
        tn = np.nanmax(np.minimum(t1, t2), -1)
        tf = np.nanmin(np.maximum(t1, t2), -1)
        hit = (tf >= tn) & (tn > 0)
        t = np.where(hit & (tn < t), tn, t)
    with np.errstate(invalid='ignore'):        # inf (no hit) stays a hole
        depth = t + rng.normal(0, noise, t.shape) * (t / 1.5) ** 2
    n = depth.size
    idx = rng.choice(n, int(outlier_frac * n), replace=False)
    depth.flat[idx] = rng.uniform(0.3, 1.4, len(idx))
    idx = rng.choice(n, int(hole_frac * n), replace=False)
    depth.flat[idx] = np.nan
    depth[~np.isfinite(depth)] = np.nan
    R_ob = _rot_z(robot[2])
    t_ob = np.array([robot[0], robot[1], 0.0])
    return dict(depth=depth.astype(np.float32), intr=(fx, fy, cx, cy, w, h),
                R_oc=R_ob @ R_bc, t_oc=R_ob @ p0 + t_ob, R_ob=R_ob, t_ob=t_ob)


class _Oak:
    def __init__(self, scene):
        self.scene = scene

    def get_depth_frame_age(self):
        return 0.0

    def get_raw_depth_frame(self):
        return self.scene['depth']

    def get_depth_intrinsics(self):
        return self.scene['intr']


def make_stepper(scene, fast):
    """An OakFloorObstacles with ROS stubbed out; .published holds the last cloud."""
    o = ofo.OakFloorObstacles.__new__(ofo.OakFloorObstacles)
    o.oak = _Oak(scene)
    o.frame = 'oak_rgb_camera_optical_frame'
    o.z_min, o.z_max, o.range_max = 0.03, 0.45, 1.6
    o.period = 0.2
    o.self_x, o.self_y = 0.37, 0.34
    o.fast = fast
    o.mem = ofo.FastFloorObstacleMemory() if fast else ofo.FloorObstacleMemory()
    o.stats = {}
    o._last_log = time.monotonic()
    o._dbg_t = time.monotonic() + 1e9      # never write /tmp/oak_floor_debug.npz
    o._floor_h = []
    o.published = None

    def _tf(target, source):
        sc = o.oak.scene
        if source == 'base_footprint':
            return sc['R_ob'], sc['t_ob']
        return sc['R_oc'], sc['t_oc']

    def publish(points):
        o.published = points
    o._tf, o.publish = _tf, publish
    return o


def _median_us(fn, n):
    fn()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        fn()
        ts.append(time.perf_counter() - t0)
    return float(np.median(ts) * 1e6)


def _cpu_pct_5hz(fn, seconds):
    period, end = 0.2, time.perf_counter() + seconds
    c0, w0 = time.process_time(), time.perf_counter()
    nxt = w0
    while nxt < end:
        time.sleep(max(0.0, nxt - time.perf_counter()))
        fn()
        nxt += period
    return 100.0 * (time.process_time() - c0) / (time.perf_counter() - w0)


def child(n, seconds):
    t_end = time.perf_counter() + 1.5          # let the core clock ramp up first
    while time.perf_counter() < t_end:
        pass
    scene = make_scene()
    import oakd_cloud
    fx, fy, cx, cy = oakd_cloud.scale_intrinsics(scene['intr'], scene['depth'].shape)
    pts = oakd_cloud.depth_to_points(scene['depth'], fx, fy, cx, cy, stride=4,
                                     z_min=0.30, z_max=1.35, max_points=12000)
    R_oc, t_oc, R_ob, t_ob = scene['R_oc'], scene['t_oc'], scene['R_ob'], scene['t_ob']
    p_odom = pts @ R_oc.T + t_oc
    p_base = (p_odom - t_ob) @ R_ob
    rng = np.random.default_rng(1)
    cand = np.repeat(p_odom[rng.choice(len(p_odom), 700, replace=False)], 2, 0)
    cand += rng.normal(0, 0.004, cand.shape)
    # Populated memory: ~2000 voxels around the robot, none forgotten per update.
    pre = t_ob + rng.uniform(-2.5, 2.5, (4000, 3)) * (1, 1, 0.04)
    rows = {}
    for fast in (False, True):
        mem_cls = ofo.FastFloorObstacleMemory if fast else ofo.FloorObstacleMemory
        mem = mem_cls()
        mem.update(pre, 0.0, t_ob[:2])
        stepper = make_stepper(scene, fast)
        r = {
            'n_points': len(pts),
            'transform': _median_us(lambda: (ofo.rigid(pts, R_oc, t_oc, fast),
                                             ofo.rigid(p_odom - t_ob, R_ob.T, None, fast)), n),
            'plane_fit': _median_us(lambda: ofo.fit_floor_plane(p_base, fast), n),
            'dedup': _median_us(lambda: ofo.min_returns_per_voxel(cand, 0.04, fast), n),
            'memory_update': _median_us(lambda: mem.update(cand, 1.0, t_ob[:2]), n),
            'memory_voxels': len(mem),
            'memory_points': _median_us(mem.points, n),
            'full_step': _median_us(stepper.step, n),
        }
        r['step_cpu_pct_5hz'] = _cpu_pct_5hz(stepper.step, seconds)
        rows['fast' if fast else 'old'] = r
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--child', action='store_true')
    ap.add_argument('-n', type=int, default=200, help='calls per median')
    ap.add_argument('--seconds', type=float, default=6.0, help='5 Hz CPU window')
    args = ap.parse_args()
    if args.child:
        print(json.dumps(child(args.n, args.seconds)))
        return
    results = {}
    for label, val in (('OPENBLAS_NUM_THREADS=1', '1'), ('OPENBLAS_NUM_THREADS unset', None)):
        env = dict(os.environ)
        env.pop('OPENBLAS_NUM_THREADS', None)
        if val is not None:
            env['OPENBLAS_NUM_THREADS'] = val
        out = subprocess.run([sys.executable, __file__, '--child', '-n', str(args.n),
                              '--seconds', str(args.seconds)],
                             env=env, check=True, capture_output=True, text=True).stdout
        results[label] = json.loads(out.strip().splitlines()[-1])
    steps = ['transform', 'plane_fit', 'dedup', 'memory_update', 'memory_points', 'full_step']
    for label, rows in results.items():
        o, f = rows['old'], rows['fast']
        print(f'\n{label}  ({o["n_points"]} points, memory {o["memory_voxels"]} voxels)')
        print(f'{"step":<16}{"old us":>10}{"fast us":>10}{"speedup":>9}')
        for s in steps:
            print(f'{s:<16}{o[s]:>10.0f}{f[s]:>10.0f}{o[s] / f[s]:>8.1f}x')
        print(f'{"step CPU @5Hz":<16}{o["step_cpu_pct_5hz"]:>9.2f}%{f["step_cpu_pct_5hz"]:>9.2f}%')


if __name__ == '__main__':
    main()
