#!/usr/bin/env python3
"""Measure Humble executor thread CPU with synthetic ROS traffic.

After sourcing ROS, run `python3 scripts/bench_rclpy_spin.py --seconds 10`.
Message rates approximate the bridge workload; payloads and callbacks are small
so the comparison mainly exercises wait-set rebuilds.
"""

import argparse
from collections import Counter
import math
import os
from pathlib import Path
import sys
from threading import Thread
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lean_rclpy import LeanNode


RATES_HZ = {
    'odom': 50,
    'odom_raw': 10,
    'tf': 50,  # Approximation; the robot profile did not record the rate.
    'scan': 10,
    'person_candidates': 10,
    'cmd_vel': 20,
    'voltage': 10,
}


def measure(label, node_type, include_raw, topics, seconds, namespace):
    sender = Node('sender', namespace=namespace)
    receiver = node_type('receiver', namespace=namespace)
    samples = Counter()
    message = Empty()

    def count(topic):
        def callback(_msg):
            samples[topic] += 1
        return callback

    for topic in topics:
        rate = RATES_HZ.get(topic)
        if rate:
            publisher = sender.create_publisher(Empty, topic, 10)
            sender.create_timer(1 / rate, lambda pub=publisher: pub.publish(message))
        if topic != 'odom_raw' or include_raw:
            receiver.create_subscription(Empty, topic, count(topic), 10)
    # Include output publishers and periodic callbacks in the receiver wait set.
    for index in range(12):
        receiver.create_publisher(Empty, f'output_{index}', 10)
    receiver.create_timer(0.1, lambda: None)
    receiver.create_timer(0.5, lambda: None)

    sending = SingleThreadedExecutor()
    receiving = SingleThreadedExecutor()
    sending.add_node(sender)
    receiving.add_node(receiver)
    publisher_thread = Thread(target=sending.spin)
    publisher_thread.start()

    def spin_until(end):
        while (remaining := end - time.monotonic()) > 0:
            receiving.spin_once(timeout_sec=remaining)

    try:
        spin_until(time.monotonic() + 2.0)
        samples.clear()
        wall_start = time.monotonic()
        cpu_start = time.thread_time()
        spin_until(wall_start + seconds)
        cpu_seconds = time.thread_time() - cpu_start
        elapsed = time.monotonic() - wall_start
        odom_hz = samples['odom'] / elapsed
        raw_hz = samples['odom_raw'] / elapsed
        if (odom_hz < 0.9 * RATES_HZ['odom']
                or (include_raw and raw_hz < 0.9 * RATES_HZ['odom_raw'])):
            raise RuntimeError(f'{label} did not receive the expected odometry load')
        print(f'{label:10} {len(list(receiver.subscriptions)):2d} subs, '
              f'{len(list(receiver.waitables)):2d} waitables, '
              f'{sum(samples.values()) / elapsed:5.1f} callbacks/s '
              f'(odom {odom_hz:.1f} Hz, raw {raw_hz:.1f} Hz), '
              f'{100 * cpu_seconds / elapsed:.2f}% of one core')
    finally:
        sending.shutdown()
        publisher_thread.join()
        receiving.shutdown()
        receiver.destroy_node()
        sender.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--subscriptions', type=int, default=16,
                        help='subscriber count in the baseline, including idle subscribers')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 1 or args.subscriptions < len(RATES_HZ):
        parser.error(f'use at least 1 second and {len(RATES_HZ)} subscriptions')
    topics = list(RATES_HZ) + [f'idle_{index}' for index in range(args.subscriptions - len(RATES_HZ))]
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init(args=[])
    try:
        for label, node_type, include_raw in (
            ('baseline', Node, True), ('qos-only', LeanNode, True), ('lean', LeanNode, False),
        ):
            measure(label, node_type, include_raw, topics, args.seconds,
                    f'/x3_rclpy_bench_{os.getpid()}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
