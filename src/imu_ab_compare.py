#!/usr/bin/env python3
"""Side-by-side A/B of the MPU9250 and the ICM-42688-P.

Subscribes to both raw IMU topics and reports rate, yaw-rate bias (the number
that made the parked robot rotate in RViz) and noise. Robot must be STATIONARY.

    python3 imu_ab_compare.py --seconds 30
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

R2DPM = 180.0 / math.pi * 60.0      # rad/s -> deg/min


class Collector(Node):
    def __init__(self, topics):
        super().__init__('imu_ab_compare')
        self.data = {t: [] for t in topics}
        self.t0 = {}
        # ONE subscription per topic. Two (e.g. a BEST_EFFORT and a reliable one)
        # both fire on every message and silently double the measured rate.
        for t in topics:
            self.create_subscription(Imu, t, self._mk(t), 100)

    def _mk(self, topic):
        def cb(msg):
            now = time.time()
            self.t0.setdefault(topic, now)
            self.data[topic].append((now,
                                     msg.angular_velocity.x,
                                     msg.angular_velocity.y,
                                     msg.angular_velocity.z,
                                     msg.linear_acceleration.x,
                                     msg.linear_acceleration.y,
                                     msg.linear_acceleration.z))
        return cb


def stats(v):
    n = len(v)
    if n == 0:
        return 0.0, 0.0
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return m, sd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--old', default='/imu/data_raw')
    ap.add_argument('--new', default='/imu/icm/data_raw')
    a = ap.parse_args()

    rclpy.init()
    node = Collector([a.old, a.new])
    print(f'Collecting {a.seconds:.0f}s. KEEP THE ROBOT STILL.\n')
    end = time.time() + a.seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

    labels = {a.old: 'MPU9250 (Rosmaster serial)', a.new: 'ICM-42688-P (i2c-7)'}
    print(f'{"":<30}{"rate":>9}{"yaw bias":>12}{"yaw noise":>12}{"|a|":>9}')
    print(f'{"":<30}{"Hz":>9}{"deg/min":>12}{"sd rad/s":>12}{"m/s^2":>9}')
    print('-' * 72)
    rows = {}
    for t in (a.old, a.new):
        d = node.data[t]
        if not d:
            print(f'{labels[t]:<30}{"NO DATA":>9}')
            continue
        span = d[-1][0] - d[0][0]
        rate = len(d) / span if span > 0 else 0.0
        gz = [r[3] for r in d]
        m, sd = stats(gz)
        norms = [math.sqrt(r[4] ** 2 + r[5] ** 2 + r[6] ** 2) for r in d]
        an, _ = stats(norms)
        rows[t] = (rate, m * R2DPM, sd, an)
        print(f'{labels[t]:<30}{rate:>9.1f}{m*R2DPM:>12.2f}{sd:>12.6f}{an:>9.3f}')

    if len(rows) == 2:
        o, n = rows[a.old], rows[a.new]
        print('-' * 72)
        print(f'{"IMPROVEMENT":<30}{o[0] and n[0]/o[0] or 0:>8.1f}x'
              f'{abs(o[1]) and abs(n[1])/abs(o[1]) or 0:>11.2f}x'
              f'{o[2] and n[2]/o[2] or 0:>11.2f}x')
        print(f'\n  yaw drift: {o[1]:+.2f} -> {n[1]:+.2f} deg/min')
        print(f'  over 30 min parked: {o[1]*30:+.0f} deg -> {n[1]*30:+.0f} deg')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
