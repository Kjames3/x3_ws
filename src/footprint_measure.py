#!/usr/bin/env python3
"""Measure the robot's outline with its own lidar: no tape measure needed.

Push each side of the robot flat against a wall in turn. The wall is then at
exactly that side's distance from the robot's centre, and the lidar reads it.
The result is in the frame the CBF uses (base centre, x forward), so it is the
number X3_CBF_FOOTPRINT wants: half length, half width.

    python3 src/footprint_measure.py

Use a flat wall at least 0.4 m tall (the lidar scans 0.32 m up) with a metre
of bare wall either side. Push until the FIRST part of the robot touches --
the ToF bracket at the nose, the widest wheel at a side -- and keep it square.
The lidar cannot see closer than 0.12 m, so it never sees the wall straight
ahead of itself; the fit uses the wall to either side, which is why it needs
that metre of bare wall.
The robot does not drive: server_x3.py holds its commands while this runs, so
the motors are released and it rolls freely. Each side takes about 3 s.
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

LASER_X = 0.055          # lidar ahead of the base centre; laser_joint yaw = pi
SIDES = (('front', 0.0, 'the NOSE'), ('rear', math.pi, 'the TAIL'),
         ('left', math.pi / 2, 'the LEFT side'), ('right', -math.pi / 2, 'the RIGHT side'))


class Reader(Node):
    def __init__(self):
        super().__init__('footprint_measure')
        self.scans = []
        # the lidar publishes best-effort; a reliable subscriber gets nothing
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        r = np.asarray(m.ranges, np.float64)
        k = np.arange(len(r))
        ok = np.isfinite(r) & (r > 0.05) & (r < 1.5)
        a = m.angle_min + k[ok] * m.angle_increment + math.pi
        self.scans.append(np.c_[LASER_X + r[ok] * np.cos(a), r[ok] * np.sin(a)])

    def collect(self, seconds):
        self.scans = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.scans


def wall_distance(points, direction):
    """Distance from the base centre to the wall facing `direction`, by a line fit.

    Points within 35 deg of that direction; the wall is the nearest straight
    run of them. Returns (distance, skew_deg, n) or None.
    """
    u = np.array([math.cos(direction), math.sin(direction)])
    along = points @ u
    across = points @ np.array([-u[1], u[0]])
    m = (along > 0.03) & (np.abs(np.arctan2(across, along)) < math.radians(35))
    if m.sum() < 15:
        return None
    along, across = along[m], across[m]
    near = along < np.percentile(along, 30) + 0.06      # the wall, not what is behind it
    for _ in range(3):                                   # line: along = a + b * across
        b, a = np.polyfit(across[near], along[near], 1)
        resid = along - (a + b * across)
        near = np.abs(resid) < 0.015
        if near.sum() < 10:
            return None
    skew = math.degrees(math.atan(b))
    if a <= 0.0 or abs(skew) > 20.0:
        return None                 # that is some other wall seen edge-on, not one facing this side
    return a / math.sqrt(1 + b * b), skew, int(near.sum())


def main():
    rclpy.init()
    n = Reader()
    out = {}
    for name, direction, label in SIDES:
        while True:
            ans = input('\nPush %s flat against the wall, hold it there, then press Enter (s = skip): ' % label)
            if ans.strip().lower() == 's':
                break
            scans = n.collect(2.5)
            vals = [v for v in (wall_distance(p, direction) for p in scans) if v]
            if len(vals) < 5:
                print('  no wall found on that side (%d scans). Is the wall tall and flat there? Try again.' % len(scans))
                continue
            d = float(np.median([v[0] for v in vals]))
            skew = float(np.median([v[1] for v in vals]))
            spread = float(np.std([v[0] for v in vals]))
            print('  %s: %.3f m from the centre  (wall skew %+.1f deg, spread %.1f mm over %d scans)' % (
                name, d, skew, spread * 1e3, len(vals)))
            if abs(skew) > 4:
                print('  the robot is %.0f deg off square; straighten it and press Enter to repeat, or type k to keep.' % abs(skew))
                if input('  > ').strip().lower() != 'k':
                    continue
            out[name] = d
            break
    print('\nresult:', ', '.join('%s %.3f' % kv for kv in out.items()))
    if all(k in out for k in ('front', 'rear')):
        print('  length %.3f m' % (out['front'] + out['rear']))
    if all(k in out for k in ('left', 'right')):
        print('  width  %.3f m' % (out['left'] + out['right']))
    if len(out) == 4:
        print('  footprint for the CBF (half length, half width): %.2f,%.2f' % (
            math.ceil(max(out['front'], out['rear']) * 100) / 100, math.ceil(max(out['left'], out['right']) * 100) / 100))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
