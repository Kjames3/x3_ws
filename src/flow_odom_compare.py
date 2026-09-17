#!/usr/bin/env python3
"""Compare PAA5100JE flow distance against wheel odometry and a tape measure.

Integrates /flow/twist and differences /odom_raw from the moment this starts,
both expressed in the robot's STARTING frame (forward / left), with heading
from /imu/data.  Drive a measured distance, stop, Ctrl+C, and it prints the
totals plus the counts_per_m, from raw sensor counts, that matches the tape.

    ros2 run flow_node first, then:
    python3 src/flow_odom_compare.py --tape-fwd 1.0          # forward run
    python3 src/flow_odom_compare.py --tape-left 1.0         # strafe-left run
"""
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

DEFAULT_COUNTS_PER_M = 23850.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Compare(Node):
    def __init__(self):
        super().__init__('flow_odom_compare')
        self.yaw = None
        self.yaw0 = None
        self.flow_t = None
        self.fx = self.fy = 0.0          # flow, start frame
        self.odom0 = None
        self.odom = None
        self.n_flow = 0
        self.peak_speed = 0.0
        self.moving_time = 0.0
        self.moving_dist = 0.0
        self.squal_min = 255
        self.low_squal = self.rejected = self.reads = 0
        self.raw_dx = self.raw_dy = 0     # sensor counts, from /flow/status
        self.create_subscription(String, '/flow/status', self.on_status, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        # Best effort to match the publisher, but a deep queue: with the default
        # sensor-data depth of 5, bursts of /flow/twist overflowed it mid-drive.
        self.create_subscription(TwistWithCovarianceStamped, '/flow/twist', self.on_flow,
                                 QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, '/odom_raw', self.on_odom, 10)
        self.create_timer(0.5, self.show)

    def on_imu(self, msg):
        self.yaw = yaw_of(msg.orientation)
        if self.yaw0 is None:
            self.yaw0 = self.yaw

    def on_flow(self, msg):
        # Integrate on the PUBLISHER's stamps, not arrival time: DDS delivers
        # /flow/twist in bursts (gaps of 0.3 s then clumps), and arrival-time
        # integration silently dropped a third of a 3 m drive.
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.flow_t is None or self.yaw is None:
            self.flow_t = now
            return
        dt, self.flow_t = now - self.flow_t, now
        if dt <= 0.0 or dt > 0.5:        # reordered, or a real publisher gap
            return
        h = wrap(self.yaw - self.yaw0)
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        self.fx += (vx * math.cos(h) - vy * math.sin(h)) * dt
        self.fy += (vx * math.sin(h) + vy * math.cos(h)) * dt
        speed = math.hypot(vx, vy)
        self.peak_speed = max(self.peak_speed, speed)
        if speed > 0.02:
            self.moving_time += dt
            self.moving_dist += speed * dt
        self.n_flow += 1

    def on_status(self, msg):
        st = json.loads(msg.data)
        self.reads += st['reads']
        self.low_squal += st['low_squal']
        self.rejected += st['rejected']
        self.squal_min = min(self.squal_min, st['squal_min'])
        self.raw_dx += st['counts_dx']
        self.raw_dy += st['counts_dy']

    def on_odom(self, msg):
        p = msg.pose.pose
        cur = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.odom0 is None:
            self.odom0 = cur
        self.odom = cur

    def odom_delta(self):
        if self.odom is None:
            return None
        x0, y0, t0 = self.odom0
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        return (dx * math.cos(t0) + dy * math.sin(t0),
                -dx * math.sin(t0) + dy * math.cos(t0),
                math.degrees(wrap(self.odom[2] - t0)))

    def show(self):
        o = self.odom_delta()
        imu = '  --  ' if self.yaw0 is None else f'{math.degrees(wrap(self.yaw - self.yaw0)):+6.1f}'
        odom = 'no /odom_raw' if o is None else f'fwd {o[0]:+.3f} left {o[1]:+.3f} m'
        print(f'flow fwd {self.fx:+.3f} left {self.fy:+.3f} m | odom {odom} | imu yaw {imu} deg'
              f' | flow msgs {self.n_flow}', flush=True)


def report(node, args):
    print('\n==== totals (start frame) ====')
    # raw is the calibration reference: summed sensor counts from /flow/status,
    # immune to message loss, but in the ROBOT frame and only whole 2 s status
    # windows (wait >= 3 s after stopping), so it suits straight runs, not arcs.
    # flow integrates /flow/twist and loses distance if messages are dropped.
    print(f'flow : fwd {node.fx:+.4f} m  left {node.fy:+.4f} m  ({node.n_flow} msgs; unreliable if low)')
    print(f'raw  : fwd {node.raw_dy / args.counts_per_m:+.4f} m  left {node.raw_dx / args.counts_per_m:+.4f} m'
          f'  ({node.raw_dy} dy / {node.raw_dx} dx counts)')
    o = node.odom_delta()
    if o:
        print(f'odom : fwd {o[0]:+.4f} m  left {o[1]:+.4f} m  yaw {o[2]:+.2f} deg')
    if node.yaw0 is not None:
        print(f'imu  : yaw {math.degrees(wrap(node.yaw - node.yaw0)):+.2f} deg')
    if node.moving_time > 0:
        print(f'speed: peak {node.peak_speed:.3f} m/s, mean while moving '
              f'{node.moving_dist / node.moving_time:.3f} m/s (datasheet: 0.4% error to 0.25 m/s)')
    if node.reads:
        print(f'squal: min {node.squal_min}, low-squal reads {node.low_squal}/{node.reads}, '
              f'rejected {node.rejected}')
    for name, tape, flow, od in (('fwd', args.tape_fwd, node.fx, o[0] if o else None),
                                 ('left', args.tape_left, node.fy, o[1] if o else None)):
        if tape is None:
            continue
        line = f'{name}: tape {tape:.3f} m | flow error {100 * (flow - tape) / tape:+.1f}%'
        raw = node.raw_dy if name == 'fwd' else node.raw_dx
        if raw:
            line += f' | raw counts_per_m {abs(raw) / tape:.0f}'
        if od is not None:
            line += f' | odom error {100 * (od - tape) / tape:+.1f}%'
        print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tape-fwd', type=float, help='measured forward distance, m')
    ap.add_argument('--tape-left', type=float, help='measured leftward distance, m')
    ap.add_argument('--counts-per-m', type=float, default=DEFAULT_COUNTS_PER_M,
                    help='value flow_node is running with')
    args, ros_args = ap.parse_known_args()    # pass --ros-args remaps through
    rclpy.init(args=['flow_odom_compare'] + ros_args)
    node = Compare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        report(node, args)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
