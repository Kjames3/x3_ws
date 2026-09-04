#!/usr/bin/env python3
"""sweep_load_bench — measure what a continuous, deskewed tilt sweep would cost.

Answers "can the Jetson take it?" WITHOUT building the pipeline first, and
without moving the servo or touching the running stack.  Every stage that
needs ROS runs in its own namespace against its own nodes, so the live
lidar_3d_processor_node / octomap / joint_state_publisher are never perturbed
and /scan is never gated.

    python3 sweep_load_bench.py baseline  [--secs 30]
    python3 sweep_load_bench.py rates     [--secs 20]
    python3 sweep_load_bench.py deskew    [--beams 2795] [--iters 200]
    python3 sweep_load_bench.py tfchain   [--rates 10,25,50,100]
    python3 sweep_load_bench.py octomap   [--rates 2.5,7.2] [--max-range 8.0]
    python3 sweep_load_bench.py servo     --i-will-stop-the-server
    python3 sweep_load_bench.py all

Stage guide
  baseline  per-process CPU from /proc deltas (NOT ps %CPU, which averages over
            the process's whole life and reads ~2x low on a freshly booted box)
  rates     achieved rate AND jitter of the topics deskew depends on.  Jitter
            is the point: TF interpolation error scales with the gap between
            tilt samples, not with their mean rate.
  deskew    the actual candidate algorithm (2 TF lookups + per-ray SLERP +
            vectorised rotate + PointCloud2 assembly) against today's single-
            transform projectLaser, at the measured beam count.
  tfchain   a PRIVATE joint_state_publisher + robot_state_publisher pair fed
            synthetic joint states at increasing rates.  Isolated on purpose:
            publishing onto the real /lidar_tilt/joint_states would fight
            server_x3.py for the topic and could gate /scan off, which is the
            F7 stale-obstacle hazard.
  octomap   a PRIVATE octomap_server fed REAL /scan_raw geometry, projected and
            re-pitched to look like sweep clouds, at controlled rates.  Reports
            CPU and whether it keeps up (insert rate vs publish rate).
  servo     Dynamixel poll-rate scaling.  The ONLY stage that needs the port,
            so it must stop x3_server; opt-in flag, and it restarts it after.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

CLK_TCK = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count() or 1

# Measured on the robot 2026-08-29 from a live /scan_raw message.  The code
# comments elsewhere say 8 Hz / 156 ms / ~1000 beams; the hardware actually
# does 7.2 Hz / 136.7 ms / 2795 beams, which is 2.8x the points.
SCAN_BEAMS = 2795
SCAN_TIME_S = 0.136667
SCAN_HZ = 7.2
ANGLE_MIN = -math.pi
ANGLE_INC = 0.002253653248772025


# ─────────────────────────── CPU sampling ────────────────────────────────

def _proc_jiffies(pid):
    try:
        with open('/proc/%d/stat' % pid) as f:
            parts = f.read().rsplit(') ', 1)[1].split()
        return int(parts[11]) + int(parts[12])   # utime + stime
    except (OSError, IndexError, ValueError):
        return None


def _proc_name(pid):
    try:
        with open('/proc/%d/comm' % pid) as f:
            return f.read().strip()
    except OSError:
        return None


def _total_jiffies():
    with open('/proc/stat') as f:
        parts = f.readline().split()[1:]
    vals = [int(v) for v in parts]
    return sum(vals), vals[3] + vals[4]   # total, idle+iowait


def _pids():
    return [int(d) for d in os.listdir('/proc') if d.isdigit()]


class CpuSampler:
    """Per-process CPU as a percentage of ONE core, from /proc jiffy deltas."""

    def __init__(self, pids=None):
        self.pids = pids
        self.t0 = None

    def start(self):
        self.t0 = time.monotonic()
        pids = self.pids if self.pids is not None else _pids()
        self.snap = {p: j for p in pids if (j := _proc_jiffies(p)) is not None}
        self.tot0, self.idle0 = _total_jiffies()

    def stop(self):
        dt = time.monotonic() - self.t0
        tot1, idle1 = _total_jiffies()
        out = []
        for pid, j0 in self.snap.items():
            j1 = _proc_jiffies(pid)
            if j1 is None:
                continue
            pct = (j1 - j0) / CLK_TCK / dt * 100.0
            if pct >= 0.05:
                out.append((pct, pid, _proc_name(pid) or '?'))
        out.sort(reverse=True)
        dtot = max(1, tot1 - self.tot0)
        busy_all = (1.0 - (idle1 - self.idle0) / dtot) * 100.0
        return out, busy_all, dt


def _find_pids(patterns):
    """PIDs whose comm OR cmdline matches any pattern."""
    hits = []
    for pid in _pids():
        try:
            with open('/proc/%d/cmdline' % pid, 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        name = _proc_name(pid) or ''
        for pat in patterns:
            if pat in name or pat in cmd:
                hits.append((pid, name, pat))
                break
    return hits


# `ros2 run` is a python wrapper that execs the node as a CHILD, so terminating
# the wrapper orphans the node.  Nineteen stray joint_state_publishers
# accumulated across bench runs that way, all publishing to the same bench
# topic, before this was caught -- the measured rates were meaningless until it
# was.  Launch the real executable directly and kill the whole process group.
ROS_LIB = os.environ.get('ROS_LIB_DIR', '/opt/ros/humble/lib')


def _exe(pkg, name):
    for base in (ROS_LIB, os.path.join(os.path.expanduser('~/x3_ws/install'),
                                       pkg, 'lib', pkg)):
        cand = os.path.join(base, pkg, name) if base == ROS_LIB \
            else os.path.join(base, name)
        if os.path.exists(cand):
            return cand
    raise FileNotFoundError('%s/%s not found' % (pkg, name))


class NodeGroup:
    """Child ROS nodes that are guaranteed to die with the bench."""

    def __init__(self):
        self.procs = []

    def spawn(self, argv, log=None):
        # Keep the node's own stderr: a bench that silently measures a dead or
        # complaining node reports zeros and looks like a hardware answer.
        out = open(log, 'wb') if log else subprocess.DEVNULL
        p = subprocess.Popen(argv, start_new_session=True,
                             stdout=out, stderr=subprocess.STDOUT)
        p._bench_log = log
        self.procs.append(p)
        return p

    def pids(self):
        return [p.pid for p in self.procs]

    def kill(self):
        for p in self.procs:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + 6.0
        for p in self.procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(p.pid), 9)
                except (ProcessLookupError, PermissionError):
                    pass
        self.procs = []


def _assert_no_strays(pattern, label):
    """Refuse to measure while a previous run's nodes are still publishing."""
    stray = [p for p, _n, _x in _find_pids([pattern]) if p != os.getpid()]
    if stray:
        raise RuntimeError(
            '%d stray %s process(es) still running (%s). They would corrupt '
            'this measurement. Kill them first:  pkill -f "%s"'
            % (len(stray), label, stray, pattern))


def _hdr(title):
    print('\n' + '=' * 72)
    print(title)
    print('=' * 72)


# ─────────────────────────── stage: baseline ─────────────────────────────

def stage_baseline(args):
    _hdr('BASELINE  — per-process CPU over %.0f s (%% of ONE core)' % args.secs)
    print('%d cores. ps %%CPU is a lifetime average and reads low on a freshly '
          'booted box; these are /proc deltas.\n' % NCPU)
    s = CpuSampler()
    s.start()
    time.sleep(args.secs)
    rows, busy_all, dt = s.stop()
    tot = sum(r[0] for r in rows)
    print('  %-7s %-22s %8s  %8s' % ('PID', 'COMM', '%1core', '%all6'))
    for pct, pid, name in rows[:18]:
        print('  %-7d %-22s %8.1f  %8.1f' % (pid, name, pct, pct / NCPU))
    print('\n  sum of sampled procs : %6.1f %% of one core  (%.2f cores)'
          % (tot, tot / 100.0))
    print('  system busy (all 6)  : %6.1f %%  => %.2f cores idle'
          % (busy_all, NCPU * (1 - busy_all / 100.0)))
    try:
        with open('/proc/loadavg') as f:
            print('  loadavg              : %s' % f.read().split(' 0')[0].strip())
    except OSError:
        pass
    return {'busy_all': busy_all, 'sum_pct': tot,
            'headroom_cores': NCPU * (1 - busy_all / 100.0)}


# ──────────────────────────── stage: rates ───────────────────────────────

def stage_rates(args):
    _hdr('RATES — what the deskew has to interpolate between')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, JointState
    from tf2_msgs.msg import TFMessage

    topics = [('/scan_raw', LaserScan, qos_profile_sensor_data),
              ('/scan', LaserScan, qos_profile_sensor_data),
              ('/lidar_tilt/joint_states', JointState, 10),
              ('/joint_states', JointState, 10),
              ('/tf', TFMessage, 10)]

    rclpy.init(args=None)
    node = Node('sweep_bench_rates')
    stamps = {t: [] for t, _, _ in topics}

    def mk(t):
        return lambda _m: stamps[t].append(time.monotonic())

    subs = [node.create_subscription(ty, t, mk(t), q) for t, ty, q in topics]
    t_end = time.monotonic() + args.secs
    while time.monotonic() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()

    print('  %-26s %7s %8s %8s %8s %8s'
          % ('topic', 'n', 'Hz', 'min ms', 'max ms', 'std ms'))
    out = {}
    for t, _, _ in topics:
        v = np.array(stamps[t])
        if len(v) < 3:
            print('  %-26s %7d %8s   (too few messages)' % (t, len(v), '-'))
            continue
        d = np.diff(v) * 1e3
        hz = 1000.0 / d.mean()
        print('  %-26s %7d %8.2f %8.1f %8.1f %8.1f'
              % (t, len(v), hz, d.min(), d.max(), d.std()))
        out[t] = (hz, d.max(), d.std())

    j = out.get('/lidar_tilt/joint_states')
    if j:
        hz, dmax, dstd = j
        print('\n  Interpolation error budget for lidar_tilt_joint:')
        print('    %-12s %10s %12s %12s' % ('sweep', 'deg/sample', 'worst gap', 'at 3 m'))
        for name, dps in (('6 s (15 deg/s)', 15.0), ('4 s (22.5)', 22.5),
                          ('2 s (45)', 45.0)):
            mean_gap = dps / hz
            worst = dps * dmax / 1e3
            print('    %-12s %10.2f %12.2f %10.1f cm'
                  % (name, mean_gap, worst,
                     300.0 * math.tan(math.radians(worst / 2.0))))
        print('    (worst gap = deg travelled across the LONGEST observed'
              ' publish gap; the\n     3 m column is the mid-gap linear-'
              'interpolation error, i.e. the deskew\n     floor unless the'
              ' tilt publisher is made faster AND regular.)')
    return out


# ─────────────────────────── stage: deskew ───────────────────────────────

def _quat_rot(q, v):
    """Rotate v (N,3) by quaternions q (N,4) as [x,y,z,w]. Vectorised."""
    qv = q[:, :3]
    qw = q[:, 3:4]
    t = 2.0 * np.cross(qv, v)
    return v + qw * t + np.cross(qv, t)


def _slerp(q0, q1, f):
    """SLERP a single pair across fractions f (N,) -> (N,4)."""
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1, d = -q1, -d
    if d > 0.9995:                       # nlerp is exact enough here
        q = q0[None, :] + f[:, None] * (q1 - q0)[None, :]
        return q / np.linalg.norm(q, axis=1, keepdims=True)
    th0 = math.acos(d)
    s0 = np.sin((1.0 - f) * th0) / math.sin(th0)
    s1 = np.sin(f * th0) / math.sin(th0)
    return s0[:, None] * q0[None, :] + s1[:, None] * q1[None, :]


def _make_scan(beams):
    rng = np.random.default_rng(0)
    r = rng.uniform(0.4, 8.0, beams).astype(np.float32)
    r[rng.random(beams) < 0.08] = np.inf       # dropouts, as the real X3 has
    return r


def _deskew(ranges, angles, q0, q1, p0, p1, max_range):
    """The candidate algorithm, start to finish, exactly as it would ship."""
    good = np.isfinite(ranges) & (ranges > 0.12) & (ranges < max_range)
    r = ranges[good]
    a = angles[good]
    f = (np.flatnonzero(good) / (len(ranges) - 1.0)).astype(np.float64)
    v = np.empty((len(r), 3), dtype=np.float64)
    v[:, 0] = r * np.cos(a)
    v[:, 1] = r * np.sin(a)
    v[:, 2] = 0.0
    q = _slerp(q0, q1, f)
    out = _quat_rot(q, v) + (p0[None, :] + f[:, None] * (p1 - p0)[None, :])
    return out.astype(np.float32)


def stage_deskew(args):
    _hdr('DESKEW — cost of the candidate algorithm at the MEASURED beam count')
    beams = args.beams
    ranges = _make_scan(beams)
    angles = (ANGLE_MIN + np.arange(beams) * ANGLE_INC).astype(np.float64)

    results = {}
    # today's path: one transform for the whole scan, via laser_geometry
    try:
        from laser_geometry import LaserProjection
        from sensor_msgs.msg import LaserScan
        proj = LaserProjection()
        msg = LaserScan()
        msg.header.frame_id = 'laser_link'
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MIN + beams * ANGLE_INC
        msg.angle_increment = ANGLE_INC
        msg.time_increment = SCAN_TIME_S / beams
        msg.scan_time = SCAN_TIME_S
        msg.range_min = 0.12
        msg.range_max = 10.0
        msg.ranges = [float(x) for x in ranges]
        t = []
        for _ in range(args.iters):
            t0 = time.perf_counter()
            proj.projectLaser(msg, range_cutoff=8.0)
            t.append(time.perf_counter() - t0)
        results['projectLaser (today)'] = np.array(t) * 1e3
    except ImportError as e:
        print('  laser_geometry unavailable (%s) — skipping the baseline' % e)

    for dps in (0.0, 15.0, 45.0):
        pitch = math.radians(dps * SCAN_TIME_S)
        q0 = np.array([0.0, 0.0, 0.0, 1.0])
        q1 = np.array([0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2)])
        p0 = np.array([0.055, 0.0, 0.340])
        p1 = p0.copy()
        t = []
        for _ in range(args.iters):
            t0 = time.perf_counter()
            pts = _deskew(ranges, angles, q0, q1, p0, p1, 8.0)
            pts.tobytes()                 # PointCloud2 payload assembly
            t.append(time.perf_counter() - t0)
        results['numpy deskew @ %.0f deg/s' % dps] = np.array(t) * 1e3

    print('  %-28s %9s %9s %9s   %s'
          % ('implementation', 'mean ms', 'p95 ms', 'max ms', '%core @7.2Hz'))
    for k, v in results.items():
        print('  %-28s %9.3f %9.3f %9.3f   %8.2f'
              % (k, v.mean(), np.percentile(v, 95), v.max(),
                 v.mean() * 1e-3 * SCAN_HZ * 100))
    print('\n  %d beams, %.1f ms scan window, %.1f Hz.  Deskew replaces'
          ' projectLaser,\n  so the marginal cost is the difference between'
          ' the two rows.' % (beams, SCAN_TIME_S * 1e3, SCAN_HZ))
    return {k: float(v.mean()) for k, v in results.items()}


# ────────────────────────── stage: tf chain ──────────────────────────────

TFCHAIN_URDF = """<?xml version="1.0"?>
<robot name="bench">
  <link name="bench_base"/>
  <link name="bench_tilt"/>
  <link name="bench_laser"/>
  <joint name="lidar_tilt_joint" type="revolute">
    <parent link="bench_base"/><child link="bench_tilt"/>
    <origin xyz="0.055 0 0.340"/><axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="2"/>
  </joint>
  <joint name="bench_laser_joint" type="fixed">
    <parent link="bench_tilt"/><child link="bench_laser"/>
    <origin rpy="0 0 3.14159"/>
  </joint>
</robot>
"""


def stage_tfchain(args):
    _hdr('TF CHAIN — cost of publishing lidar_tilt_joint faster')
    print('  Private namespace /bench: its own joint_state_publisher +\n'
          '  robot_state_publisher.  The real /lidar_tilt/joint_states is NOT\n'
          '  touched — a second publisher there would fight server_x3.py for\n'
          '  the /scan safety gate.\n')

    _assert_no_strays('__ns:=/bench', 'bench TF node')

    urdf = '/tmp/sweep_bench.urdf'
    with open(urdf, 'w') as f:
        f.write(TFCHAIN_URDF)

    group = NodeGroup()
    try:
        rsp = group.spawn(
            [_exe('robot_state_publisher', 'robot_state_publisher'), urdf,
             '--ros-args', '-r', '__ns:=/bench',
             '-r', '/tf:=/bench/tf', '-r', '/tf_static:=/bench/tf_static'])
        jsp = group.spawn(
            [_exe('joint_state_publisher', 'joint_state_publisher'),
             '--ros-args', '-r', '__ns:=/bench',
             '-p', "source_list:=['/bench/tilt_states']",
             '-p', 'rate:=%g' % args.jsp_rate])
        time.sleep(4.0)
        pids = {rsp.pid: 'robot_state_pub', jsp.pid: 'joint_state_pub'}
        for pid in list(pids):
            if _proc_jiffies(pid) is None:
                print('  bench node %d died on startup' % pid)
                return {}
        print('  bench nodes: %s   (jsp rate param = %g Hz)\n' % (
            ', '.join('%s(%d)' % (n, p) for p, n in pids.items()),
            args.jsp_rate))

        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage

        rclpy.init(args=None)
        node = Node('sweep_bench_tfchain')
        pub = node.create_publisher(JointState, '/bench/tilt_states', 10)
        tf_n = [0]
        node.create_subscription(TFMessage, '/bench/tf',
                                 lambda _m: tf_n.__setitem__(0, tf_n[0] + 1), 50)
        time.sleep(1.0)

        print('  %-10s %10s %10s %10s %10s'
              % ('cmd Hz', 'js Hz', 'tf Hz', 'jsp %core', 'rsp %core'))
        out = {}
        for hz in [float(x) for x in args.rates.split(',')]:
            s = CpuSampler(list(pids))
            tf_n[0] = 0
            n_sent = 0
            s.start()
            t_end = time.monotonic() + args.secs
            period = 1.0 / hz
            nxt = time.monotonic()
            while time.monotonic() < t_end:
                m = JointState()
                m.header.stamp = node.get_clock().now().to_msg()
                m.name = ['lidar_tilt_joint']
                m.position = [0.0]
                m.velocity = [0.0]
                pub.publish(m)
                n_sent += 1
                nxt += period
                # spin_once returns as soon as ONE callback runs, not after
                # the full timeout, so a single call does NOT pace the loop
                # when the incoming rate is high -- it free-runs.  Spin until
                # the deadline actually passes.
                while True:
                    slack = nxt - time.monotonic()
                    if slack <= 0:
                        break
                    rclpy.spin_once(node, timeout_sec=slack)
            rows, _, dt = s.stop()
            by = {p: pct for pct, p, _ in rows}
            jsp = sum(v for p, v in by.items() if 'joint' in pids.get(p, ''))
            rsp = sum(v for p, v in by.items() if 'robot' in pids.get(p, ''))
            out[hz] = (n_sent / dt, tf_n[0] / dt, jsp, rsp)
            print('  %-10.0f %10.1f %10.1f %10.1f %10.1f'
                  % (hz, n_sent / dt, tf_n[0] / dt, jsp, rsp))
        node.destroy_node()
        rclpy.shutdown()
        print('\n  (jsp/rsp %core are on top of each other; the real stack also'
              '\n   carries wheel joints, so treat these as a lower bound.)')
        return out
    finally:
        group.kill()


# ─────────────────────────── stage: octomap ──────────────────────────────

def stage_octomap(args):
    _hdr('OCTOMAP — the load that actually scales with sweep rate')
    print('  Private /bench octomap_server fed REAL /scan_raw geometry,\n'
          '  re-pitched through the sweep range so the raycasts are as long\n'
          '  and as varied as a real sweep.  The live octomap is untouched.\n')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from std_msgs.msg import Header
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    _assert_no_strays('bench_octomap', 'bench octomap_server')

    rclpy.init(args=None)
    node = Node('sweep_bench_octomap')

    # 1. grab real scans
    grabbed = []
    node.create_subscription(LaserScan, '/scan_raw',
                             lambda m: grabbed.append(m), qos_profile_sensor_data)
    t_end = time.monotonic() + 8.0
    while time.monotonic() < t_end and len(grabbed) < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not grabbed:
        print('  no /scan_raw — is the lidar up?  aborting this stage.')
        node.destroy_node()
        rclpy.shutdown()
        return {}
    print('  captured %d real scans (%d beams each)'
          % (len(grabbed), len(grabbed[0].ranges)))

    # 2. build a bank of pitched clouds from them
    bank = []
    for k, msg in enumerate(grabbed):
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        good = np.isfinite(r) & (r > 0.35) & (r < args.max_range)
        r, a = r[good], a[good]
        v = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        pitch = math.radians(-45.0 + 90.0 * (k / max(1, len(grabbed) - 1)))
        c, s = math.cos(pitch), math.sin(pitch)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        bank.append((v @ R.T).astype(np.float32))
    npts = int(np.mean([len(b) for b in bank]))
    print('  bank: %d clouds, %d points mean, pitched -45..+45 deg' % (len(bank), npts))
    print('  raycast work per cloud ~ %d pts x %.0f voxels = %.2f M traversals'
          % (npts, args.max_range / 0.05, npts * (args.max_range / 0.05) / 1e6))

    fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z'))]

    def cloud_msg(arr, stamp):
        m = PointCloud2()
        m.header = Header(stamp=stamp, frame_id='bench_laser')
        m.height = 1
        m.width = len(arr)
        m.fields = fields
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * len(arr)
        m.is_dense = True
        m.data = arr.tobytes()
        return m

    group = NodeGroup()
    try:
        group.spawn(
            [_exe('tf2_ros', 'static_transform_publisher'),
             '--x', '0.055', '--z', '0.340',
             '--frame-id', 'bench_odom', '--child-frame-id', 'bench_laser'],
            log='/tmp/bench_stf.log')
        octo = group.spawn(
            [_exe('octomap_server', 'octomap_server_node'),
             '--ros-args', '-r', '__node:=bench_octomap',
             '-r', 'cloud_in:=/bench/cloud',
             '-r', '/octomap_binary:=/bench/octomap_binary',
             '-r', '/octomap_full:=/bench/octomap_full',
             '-r', '/octomap_point_cloud_centers:=/bench/centers',
             '-r', '/occupied_cells_vis_array:=/bench/vis',
             '-r', '/projected_map:=/bench/projected_map',
             '-p', 'frame_id:=bench_odom',
             '-p', 'base_frame_id:=bench_odom',
             # %.6f, NOT %g: %g renders 8.0 as "8", which the CLI parses as
             # an INTEGER, and octomap_server aborts with
             # InvalidParameterTypeException (SIGABRT, rc=-6) rather than
             # warning.  With the node's stderr sent to DEVNULL this reads as
             # "octomap did 0 work at 0% CPU" -- a plausible-looking hardware
             # answer that is entirely a formatting bug.
             '-p', 'resolution:=%.6f' % args.resolution,
             '-p', 'sensor_model.max_range:=%.6f' % args.max_range,
             '-p', 'filter_speckles:=true',
             '-p', 'compress_map:=true',
             '-p', 'publish_free_space:=false'], log='/tmp/bench_octomap.log')
        time.sleep(6.0)
        oct_pids = [octo.pid]
        if octo.poll() is not None or _proc_jiffies(octo.pid) is None:
            print('  bench octomap_server exited (rc=%s):' % octo.poll())
            print(open('/tmp/bench_octomap.log').read()[-1500:])
            return {}
        print('  bench octomap pid %s\n' % oct_pids)

        pub = node.create_publisher(PointCloud2, '/bench/cloud', 5)
        from octomap_msgs.msg import Octomap
        ins = [0]
        node.create_subscription(Octomap, '/bench/octomap_binary',
                                 lambda _m: ins.__setitem__(0, ins[0] + 1), 5)
        time.sleep(1.5)

        print('  %-10s %10s %10s %10s %10s'
              % ('pub Hz', 'sent', 'inserted', 'keep up?', 'oct %core'))
        out = {}
        for hz in [float(x) for x in args.rates.split(',')]:
            s = CpuSampler(oct_pids)
            ins[0] = 0
            sent = 0
            s.start()
            period = 1.0 / hz
            nxt = time.monotonic()
            t_end = time.monotonic() + args.secs
            while time.monotonic() < t_end:
                pub.publish(cloud_msg(bank[sent % len(bank)],
                                      node.get_clock().now().to_msg()))
                sent += 1
                nxt += period
                while True:
                    slack = nxt - time.monotonic()
                    if slack <= 0:
                        break
                    rclpy.spin_once(node, timeout_sec=slack)
            time.sleep(1.0)
            rclpy.spin_once(node, timeout_sec=0.2)
            rows, _, dt = s.stop()
            cpu = sum(pct for pct, _, _ in rows)
            keep = ins[0] / max(1, sent)
            out[hz] = (sent / dt, ins[0] / dt, keep, cpu)
            print('  %-10.1f %10.1f %10.1f %9.0f%% %10.1f'
                  % (hz, sent / dt, ins[0] / dt, keep * 100, cpu))
        if all(v[1] < 0.1 for v in out.values()):
            print('\n  ZERO inserts — diagnostics:')
            print('    /bench/cloud subscribers seen by us: %d'
                  % pub.get_subscription_count())
            print('    octomap alive: %s' % (octo.poll() is None))
            try:
                print('    --- octomap log tail ---')
                print('    ' + open('/tmp/bench_octomap.log').read()[-800:]
                      .replace('\n', '\n    '))
            except OSError:
                pass
        node.destroy_node()
        rclpy.shutdown()
        print('\n  "keep up?" below ~95%% means octomap_server is dropping'
              ' clouds: it is\n  single-threaded, so the message filter backs'
              ' up and you get the\n  "queue is full" symptom rather than an'
              ' error.')
        return out
    finally:
        group.kill()


# ──────────────────────────── stage: servo ───────────────────────────────

def _svc(action):
    """systemctl on x3_server. Password from SUDO_PASS if sudo needs one."""
    pw = os.environ.get('SUDO_PASS')
    cmd = ['sudo', '-S', 'systemctl', action, 'x3_server']
    r = subprocess.run(cmd, input=(pw + '\n') if pw else '',
                       capture_output=True, text=True)
    print('  systemctl %s x3_server -> rc=%d %s'
          % (action, r.returncode, (r.stderr or '').strip()[-120:]))
    return r.returncode == 0


def stage_servo(args):
    _hdr('SERVO — can the Dynamixel bus feed a 50 Hz tilt publisher?')
    if not args.i_will_stop_the_server:
        print('  SKIPPED.  This stage needs /dev/openrb150, which x3_server\n'
              '  holds.  Re-run with --i-will-stop-the-server to stop the\n'
              '  service, measure, and restart it.')
        return {}
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    _svc('stop')
    time.sleep(3.0)
    try:
        from dynamixel_servo import XL430  # noqa
        servo = XL430('/dev/openrb150')
        sid = 1
        print('  port /dev/openrb150 @ %d baud, servo id %d' % (servo.baud, sid)
              if hasattr(servo, 'baud') else '  port /dev/openrb150, id %d' % sid)
        out = {}
        print('  %-10s %10s %10s %12s %10s'
              % ('target Hz', 'achieved', 'read ms', 'reads/cycle', '%core'))
        for hz, nread in ((10, 2), (25, 2), (50, 2), (50, 1), (100, 1)):
            s = CpuSampler([os.getpid()])
            s.start()
            n, lat = 0, []
            t_end = time.monotonic() + args.secs
            period = 1.0 / hz
            nxt = time.monotonic()
            while time.monotonic() < t_end:
                t0 = time.perf_counter()
                servo.read_pos(sid)
                if nread > 1:
                    servo.read_moving(sid)
                lat.append((time.perf_counter() - t0) * 1e3)
                n += 1
                nxt += period
                sl = nxt - time.monotonic()
                if sl > 0:
                    time.sleep(sl)
            rows, _, dt = s.stop()
            cpu = sum(p for p, _, _ in rows)
            out[(hz, nread)] = (n / dt, float(np.mean(lat)), cpu)
            print('  %-10d %10.1f %10.2f %12d %10.1f%s'
                  % (hz, n / dt, float(np.mean(lat)), nread, cpu,
                     '   <-- BUS SATURATED' if n / dt < hz * 0.9 else ''))
        try:
            servo.close()
        except Exception:
            pass
        return out
    except Exception as e:
        print('  servo stage failed: %s' % e)
        return {}
    finally:
        _svc('start')
        time.sleep(2.0)
        st = subprocess.run(['systemctl', 'is-active', 'x3_server'],
                            capture_output=True, text=True).stdout.strip()
        print('\n  x3_server restarted -> %s' % st)


# ──────────────────────────────── main ───────────────────────────────────

STAGES = {'baseline': stage_baseline, 'rates': stage_rates,
          'deskew': stage_deskew, 'tfchain': stage_tfchain,
          'octomap': stage_octomap, 'servo': stage_servo}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('stage', choices=list(STAGES) + ['all'])
    ap.add_argument('--secs', type=float, default=15.0,
                    help='sampling window per measurement point')
    ap.add_argument('--beams', type=int, default=SCAN_BEAMS)
    ap.add_argument('--iters', type=int, default=200)
    ap.add_argument('--rates', default=None,
                    help='comma list; tfchain Hz or octomap cloud Hz')
    ap.add_argument('--max-range', type=float, default=8.0)
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--jsp-rate', type=float, default=10.0,
                    help="joint_state_publisher's own republish rate (its "
                         "`rate` param; ROS default 10)")
    ap.add_argument('--i-will-stop-the-server', action='store_true')
    args = ap.parse_args()

    order = ['baseline', 'rates', 'deskew', 'tfchain', 'octomap', 'servo'] \
        if args.stage == 'all' else [args.stage]
    for name in order:
        defaults = {'tfchain': '10,25,50,100', 'octomap': '2.5,7.2,14.4'}
        a = argparse.Namespace(**vars(args))
        if a.rates is None:
            a.rates = defaults.get(name, '10,25,50')
        try:
            STAGES[name](a)
        except KeyboardInterrupt:
            print('\n  interrupted')
            break
        except Exception as e:
            print('\n  stage %s FAILED: %s: %s' % (name, type(e).__name__, e))
            import traceback
            traceback.print_exc()


if __name__ == '__main__':
    main()
