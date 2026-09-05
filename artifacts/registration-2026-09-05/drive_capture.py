"""Stop-and-go multi-pose capture for the 3D registration study.

You drive; the script decides when to sweep.  It watches /odom and fires a
sweep automatically once the robot has moved at least MIN_MOVE_M from the last
station AND then held still for STILL_S.  So the loop is simply:

    drive somewhere  ->  let go of the stick  ->  it sweeps  ->  drive on

No interlock is touched.  `_tilt_nav_conflict()` still aborts a sweep the
moment the stick moves, which here is a feature: if you nudge the robot
mid-sweep the station is discarded and retaken rather than silently recorded
with chassis motion smeared into it.

WHAT THIS FIXES ABOUT THE EARLIER CAPTURES
The 2026-09-04/05 npz files concatenate every point with no cloud boundary and
no per-cloud stamp.  That forced the acquisition tilt of every return to be
reconstructed geometrically in the offline study, and made "how old is this
obstacle" unanswerable in the virtual-scan study.  Here each point carries a
`cloud_id`, and every cloud carries its stamp and its station -- one extra
index array, both questions answerable.

Usage on the robot:

    python3 drive_capture.py                 # 8 stations, default
    python3 drive_capture.py --stations 12
    python3 drive_capture.py --probe         # preflight only, never moves

Motors are left ENABLED (you are driving). They are not disabled at exit for
that reason; stop the robot yourself when done.
"""

import argparse
import asyncio
import json
import os
import threading
import time
from collections import Counter, OrderedDict

import numpy as np
import rclpy
import tf2_ros
import websockets
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2

from yahboomcar_bringup.lidar_deskew import rotate

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
MIN_MOVE_M = 0.45        # a station must be meaningfully away from the last
STILL_S = 2.5            # hold still this long to trigger (> the 1.5 s lockout)
STILL_EPS_M = 0.02       # odom jitter that still counts as stopped
SWEEP_S = 10.0           # ~2.5 ping-pong cycles at 45 deg/s
SWEEP_SPEED = 45.0

rclpy.init()
node = Node('registration_drive_capture')
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, node)

counts = Counter()
pending = []
joints = []
odom = []                # (t, x, y, yaw)
cmd_max = 0.0
station = -1             # -1 = not recording
cloud_points, cloud_ids, cloud_meta = [], [], []
stations = []          # (index, x, y, yaw) captured at sweep start
_next_cloud_id = 0


def key(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def on_joint(m):
    if 'lidar_tilt_joint' not in m.name:
        return
    i = m.name.index('lidar_tilt_joint')
    joints.append([key(m) * 1e-9, m.position[i], float(m.velocity[i] > .5)])


def on_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y ** 2 + q.z ** 2))
    odom.append([time.monotonic(), p.x, p.y, float(yaw)])


_last_nonzero_cmd = float('-inf')


def on_cmd(m):
    global cmd_max, _last_nonzero_cmd
    mag = max(abs(m.linear.x), abs(m.linear.y), abs(m.angular.z))
    cmd_max = max(cmd_max, mag)
    # server_x3._enqueue_motion stamps _last_teleop_motion_t on ANY nonzero
    # command, including one too small to overcome the motor deadband. A
    # joystick resting slightly off-centre therefore holds the interlock shut
    # while odom reads perfectly still -- which is exactly how the first run
    # died at station 6. Gate on the same signal the server gates on.
    if mag > 0.0:
        _last_nonzero_cmd = time.monotonic()


def on_cloud(m):
    counts['cloud'] += 1
    pending.append(m)


def on_scan(m):
    counts['scan'] += 1


def process():
    """Transform each cloud to base_footprint and keep it WITH its identity."""
    global _next_cloud_id
    while pending:
        m = pending[0]
        try:
            t = buf.lookup_transform('base_footprint', m.header.frame_id,
                                     Time.from_msg(m.header.stamp))
        except tf2_ros.TransformException:
            if len(pending) > 10:
                pending.pop(0)
                counts['tf_failed'] += 1
                continue
            return
        pending.pop(0)
        if station < 0:
            continue                      # not recording between stations
        xyz = np.frombuffer(m.data, dtype='<f4').reshape(-1, 3).copy()
        q, v = t.transform.rotation, t.transform.translation
        local = rotate(xyz, np.array([q.x, q.y, q.z, q.w])) + \
            np.array([v.x, v.y, v.z])
        cloud_points.append(local.astype(np.float32))
        cloud_ids.append(np.full(len(local), _next_cloud_id, dtype=np.int32))
        cloud_meta.append([_next_cloud_id, station, key(m) * 1e-9, len(local)])
        _next_cloud_id += 1
        counts['recorded'] += 1


for typ, topic, cb, qos in [
        (PointCloud2, '/pointcloud_raw', on_cloud, 10),
        (JointState, '/lidar_tilt/joint_states', on_joint, 10),
        (LaserScan, '/scan', on_scan, qos_profile_sensor_data),
        (Twist, '/cmd_vel', on_cmd, 10),
        (Odometry, '/odom', on_odom, 10)]:
    node.create_subscription(typ, topic, cb, qos)
node.create_timer(0.03, process)

_stop = False


def spin():
    while not _stop:
        rclpy.spin_once(node, timeout_sec=0.05)


def pose_now():
    return np.array(odom[-1][1:]) if odom else None


def still_since():
    """How long the robot has been within STILL_EPS_M, in seconds."""
    if len(odom) < 2:
        return 0.0
    here = np.array(odom[-1][1:3])
    for i in range(len(odom) - 1, -1, -1):
        if np.linalg.norm(np.array(odom[i][1:3]) - here) > STILL_EPS_M:
            return odom[-1][0] - odom[i][0]
    return odom[-1][0] - odom[0][0]


class SweepRefused(RuntimeError):
    """The server declined to start a sweep. Recoverable: wait and retake."""


async def drain(ws, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), 0.3))
            if m.get('type') == '3d_scan_status' and m.get('refused'):
                raise SweepRefused(m.get('reason', 'unknown'))
        except asyncio.TimeoutError:
            pass


async def main(n_stations, probe):
    global station, cmd_max
    async with websockets.connect('ws://localhost:8081', max_size=None) as ws:
        await ws.send(json.dumps({'type': 'set_sweep_config',
                                  'mode': 'continuous', 'speed': SWEEP_SPEED}))
        await drain(ws, 4)
        print('preflight: %s' % dict(counts), flush=True)
        if not odom or len(joints) < 10:
            raise RuntimeError('live inputs missing (odom/joints)')
        print('preflight OK. Drive with the controller.', flush=True)
        if probe:
            return

        last = pose_now()[:2]
        taken = 0
        while taken < n_stations:
            here = pose_now()[:2]
            moved = np.linalg.norm(here - last)
            quiet = time.monotonic() - _last_nonzero_cmd
            if moved < MIN_MOVE_M or still_since() < STILL_S or quiet < STILL_S:
                if moved >= MIN_MOVE_M and still_since() >= STILL_S and quiet < STILL_S:
                    if int(time.monotonic()) % 5 == 0:
                        print('  waiting: stick still sending commands '
                              '(%.1f s quiet, need %.1f) -- check for drift'
                              % (quiet, STILL_S), flush=True)
                await asyncio.sleep(0.25)
                continue

            station = taken
            cmd_max = 0.0
            start_pose = pose_now()
            n0 = counts['recorded']
            print('\n[station %d/%d] parked %.2f m from the last one at '
                  '(%.2f, %.2f) -- sweeping %.0f s, HANDS OFF'
                  % (taken + 1, n_stations, moved, start_pose[0],
                     start_pose[1], SWEEP_S), flush=True)
            try:
                await ws.send(json.dumps({'type': 'toggle_3d_scan',
                                          'enabled': True}))
                await drain(ws, SWEEP_S)
            except SweepRefused as e:
                station = -1
                print('  REFUSED (%s) -- letting the interlock clear, retaking.'
                      % e, flush=True)
                await ws.send(json.dumps({'type': 'toggle_3d_scan',
                                          'enabled': False}))
                try:
                    await drain(ws, 3.0)
                except SweepRefused:
                    pass
                continue
            await ws.send(json.dumps({'type': 'toggle_3d_scan', 'enabled': False}))
            try:
                await drain(ws, 2.5)
            except SweepRefused:
                pass
            station = -1

            end_pose = pose_now()
            drift = float(np.linalg.norm(end_pose[:2] - start_pose[:2]))
            got = counts['recorded'] - n0
            if cmd_max > 0.01 or drift > 0.03:
                print('  DISCARDED: robot moved during the sweep '
                      '(cmd %.3f, drift %.3f m). Retaking.'
                      % (cmd_max, drift), flush=True)
                for i in range(len(cloud_meta) - 1, -1, -1):
                    if cloud_meta[i][1] == taken:
                        cloud_meta.pop(i)
                        cloud_points.pop(i)
                        cloud_ids.pop(i)
                continue
            if got < 20:
                print('  DISCARDED: only %d clouds. Retaking.' % got, flush=True)
                continue
            stations.append([taken, float(start_pose[0]), float(start_pose[1]),
                             float(start_pose[2])])
            print('  kept %d clouds, drift %.3f m. Drive to the next spot.'
                  % (got, drift), flush=True)
            last = here
            taken += 1
        print('\nall %d stations captured.' % n_stations, flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--stations', type=int, default=8)
    ap.add_argument('--probe', action='store_true')
    args = ap.parse_args()

    th = threading.Thread(target=spin)
    th.start()
    try:
        asyncio.run(main(args.stations, args.probe))
    finally:
        _stop = True
        th.join()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        if cloud_points:
            path = os.path.join(OUT_DIR, 'drive_capture.npz')
            np.savez_compressed(
                path,
                points=np.concatenate(cloud_points),
                cloud_id=np.concatenate(cloud_ids),
                cloud_meta=np.array(cloud_meta, dtype=float),
                joints=np.array(joints),
                odom=np.array(odom),
                stations=np.array(stations, dtype=float))
            print('wrote %s: %d points, %d clouds, %d stations'
                  % (path, sum(len(c) for c in cloud_points), len(cloud_meta),
                     len({int(m[1]) for m in cloud_meta})), flush=True)
        print('counts: %s' % dict(counts), flush=True)
