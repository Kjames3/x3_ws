#!/usr/bin/env python3
"""Measure the lidar's yaw offset from the direction the robot really drives.

Drives straight forward and back and compares two views of the same move:
  - wheel odometry (/odom): the direction the base travelled,
  - the lidar: the displacement that lines up the scan taken before the move
    with the one taken after, both with the robot parked.
If the lidar sat exactly at its nominal yaw (180 deg, laser_joint) the two
directions would agree. The difference is the mount's yaw error.

*** THE ROBOT DRIVES ITSELF and this bypasses the CBF: 0.9 m forward and back,
    --legs times. Clear 1.2 m ahead. Leave the room as it is while it runs. ***

    python3 src/lidar_yaw_calib.py --yes-path-is-clear

Prints the offset to add to a lidar bearing (X3_LIDAR_YAW_OFFSET_DEG in
server_x3.py, and the yaw of laser_joint in the URDF). With --use-tf the scans
are placed by the TF tree instead of the nominal 180 deg, so after the URDF is
corrected the offset it reports should be near zero.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

LASER_X = 0.0435         # as server_x3._scan_cb


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def icp(src, dst, guess, iters=40, reject=0.12):
    """Rigid 2D transform taking src onto dst: p_dst = R(w) p_src + (x, y)."""
    x, y, w = guess
    tree = cKDTree(dst)
    frac = rms = 0.0
    for _ in range(iters):
        c, s = math.cos(w), math.sin(w)
        p = np.c_[c * src[:, 0] - s * src[:, 1] + x, s * src[:, 0] + c * src[:, 1] + y]
        d, j = tree.query(p)
        m = d < reject
        if m.sum() < 100:
            return None
        a, b = src[m], dst[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[1] *= -1
            r = vt.T @ u.T
        w = math.atan2(r[1, 0], r[0, 0])
        x, y = cb - r @ ca
        frac, rms = float(m.mean()), float(np.sqrt(np.mean(d[m] ** 2)))
    return x, y, w, frac, rms


class Calib(Node):
    def __init__(self, applied_deg, use_tf=False):
        super().__init__('lidar_yaw_calib')
        self.applied = math.radians(applied_deg)
        self.tf = self.tf_yaw_deg = None
        if use_tf:
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self._tf_listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scans, self.odom = [], None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)

    def on_scan(self, m):
        r = np.asarray(m.ranges, np.float64)
        k = np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.15) & (r < 6.0)
        yaw, x0, y0 = math.pi, LASER_X, 0.0
        if self.tf is not None:
            # where the robot's own TF tree (the URDF) says the lidar is
            try:
                t = self.tf.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time()).transform
            except Exception:
                return
            yaw, x0, y0 = yaw_of(t.rotation), t.translation.x, t.translation.y
            self.tf_yaw_deg = math.degrees(yaw)
        a = m.angle_min + k[ok] * m.angle_increment + yaw + self.applied
        self.scans.append(np.c_[x0 + r[ok] * np.cos(a), y0 + r[ok] * np.sin(a)])

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def run(self, vx, seconds):
        t = Twist()
        t.linear.x = float(vx)
        end, nxt = time.monotonic() + seconds, 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= nxt:
                self.pub.publish(t)
                nxt = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def parked_scan(self):
        """Stand still, then merge 6 scans (parked: no skew, less noise)."""
        self.run(0.0, 1.5)
        self.scans = []
        self.run(0.0, 1.2)
        pts = np.vstack(self.scans[-6:])
        return pts[::3], self.odom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yes-path-is-clear', action='store_true')
    ap.add_argument('--legs', type=int, default=2, help='forward+back pairs')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--metres', type=float, default=0.9)
    ap.add_argument('--applied-deg', type=float, default=0.0,
                    help='offset already known: rotate the scans by it and report what is left')
    ap.add_argument('--use-tf', action='store_true',
                    help='place the lidar by the TF tree (URDF) instead of the nominal 180 deg: checks a corrected URDF')
    a = ap.parse_args()
    if not a.yes_path_is_clear:
        raise SystemExit('refusing to drive: pass --yes-path-is-clear')
    rclpy.init()
    n = Calib(a.applied_deg, a.use_tf)
    rows = []
    try:
        n.run(0.0, 1.5)                      # let server_x3 notice and yield
        prev, od0 = n.parked_scan()
        for leg in range(2 * a.legs):
            v = a.speed if leg % 2 == 0 else -a.speed
            n.run(v, a.metres / a.speed)
            cur, od1 = n.parked_scan()
            c, s = math.cos(od0[2]), math.sin(od0[2])
            ox, oy = od1[0] - od0[0], od1[1] - od0[1]
            guess = (c * ox + s * oy, -s * ox + c * oy, od1[2] - od0[2])
            res = icp(cur, prev, guess)
            if res is None or res[3] < 0.6:
                print('leg %d: scan match failed (%s); skipped' % (leg + 1, 'no overlap' if res is None else 'inliers %.0f%%' % (100 * res[3])))
            else:
                odom_dir = math.atan2(guess[1], guess[0])
                lidar_dir = math.atan2(res[1], res[0])
                delta = math.degrees((odom_dir - lidar_dir + math.pi) % (2 * math.pi) - math.pi)
                rows.append((leg + 1, 'fwd' if v > 0 else 'back', math.hypot(guess[0], guess[1]), math.hypot(res[0], res[1]),
                             math.degrees(guess[2]), math.degrees(res[2]), delta, res[3], res[4]))
            prev, od0 = cur, od1
    finally:
        n.run(0.0, 0.5)
    print('%-4s %-5s %10s %10s %10s %10s %12s %8s %7s' % ('leg', 'dir', 'odom m', 'lidar m', 'odom yaw', 'lidar yaw', 'offset deg', 'inliers', 'rms mm'))
    for r in rows:
        print('%-4d %-5s %10.3f %10.3f %10.2f %10.2f %12.2f %7.0f%% %7.1f' % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], 100 * r[7], 1e3 * r[8]))
    if n.tf_yaw_deg is not None:
        print('\nTF says the lidar is at %.2f deg yaw (nominal 180)' % (n.tf_yaw_deg % 360))
    if rows:
        d = np.array([r[6] for r in rows])
        print('\nlidar yaw offset %s: %+.2f deg (mean of %d legs, spread %.2f)' % (
            'still left' if (a.applied_deg or a.use_tf) else 'to add to a lidar bearing', d.mean(), len(d), d.std()))
        print('distance, lidar / odometry: %.3f' % np.mean([r[3] / r[2] for r in rows]))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
