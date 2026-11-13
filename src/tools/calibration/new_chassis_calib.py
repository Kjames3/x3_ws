"""Encoder scale and yaw lever arm for the new chassis, driven by hand.

Drive with the GUI/controller while this records; it needs no motor access.

  straight : press Enter at the start mark, drive to the end mark (any speed,
             stop fully), press Enter. counts_per_m = forward counts / distance.
  spin     : press Enter, turn in place N full turns (stop roughly where you
             began), press Enter. Turn angle comes from the integrated
             /imu/data_raw gyro, so stopping exactly is not needed.
             yaw_wheel_separation = rim travel per wheel / turn angle.

Usage (on the robot, x3_server running):
  python3 src/tools/calibration/new_chassis_calib.py straight --distance 2.4
  python3 src/tools/calibration/new_chassis_calib.py spin --counts-per-m <from straight>

/wheel_encoders positions are logical FL, RL, FR, RR, already sign-corrected
by the driver, and DEcrease driving forward.
"""

import argparse
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry


class Rec(Node):
    def __init__(self):
        super().__init__('new_chassis_calib')
        self.counts = None
        self.yaw = 0.0
        self._t = None
        self.odom = None
        self.create_subscription(JointState, '/wheel_encoders', self._enc, 50)
        self.create_subscription(Imu, '/imu/data_raw', self._imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, 20)

    def _enc(self, m):
        self.counts = list(m.position)

    def _imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self._t is not None and 0.0 < t - self._t < 0.1:
            self.yaw += m.angular_velocity.z * (t - self._t)
        self._t = t

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def snap(self):
        return list(self.counts), self.yaw, self.odom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['straight', 'spin'])
    ap.add_argument('--distance', type=float, default=2.4, help='straight: metres between the marks')
    ap.add_argument('--counts-per-m', type=float, default=7624.0, help='spin: value from the straight run (7624, 2026-11-13)')
    a = ap.parse_args()

    rclpy.init()
    node = Rec()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    import time
    t0 = time.time()
    while node.counts is None or node._t is None:
        if time.time() - t0 > 10:
            raise SystemExit('no /wheel_encoders or /imu/data_raw after 10 s (is x3_server up? ROS_DOMAIN_ID=42?)')
        time.sleep(0.1)

    input('Robot at the start, still. Press Enter, then drive... ')
    c0, y0, o0 = node.snap()
    input('Stopped at the end? Press Enter. ')
    time.sleep(0.3)
    c1, y1, o1 = node.snap()
    d = [-(b - a_) for a_, b in zip(c0, c1)]          # + = forward rim travel, counts
    fl, rl, fr, rr = d
    turn = y1 - y0
    print('wheel counts FL RL FR RR:', [round(v) for v in d])
    print('imu turn: %.1f deg' % math.degrees(turn))
    if o0 and o1:
        print('odom moved %.3f m (with the current counts_per_m)' % math.hypot(o1[0] - o0[0], o1[1] - o0[1]))
    if a.mode == 'straight':
        fwd = (fl + rl + fr + rr) / 4.0
        cpm = abs(fwd) / a.distance
        print('per-wheel counts/m:', [round(abs(v) / a.distance) for v in d])
        print('==> counts_per_m = %.0f  (forward mean over %.3f m; heading changed %.1f deg)'
              % (cpm, a.distance, math.degrees(turn)))
    else:
        rim = (-fl - rl + fr + rr) / 4.0 / a.counts_per_m   # m, + = left turn
        if abs(turn) < math.radians(90):
            raise SystemExit('turn too small to measure; do at least one full turn')
        print('==> yaw_wheel_separation = %.3f m  (rim %.3f m over %.1f deg; geometric 0.681)'
              % (rim / turn, rim, math.degrees(turn)))
    # Exit without tearing rclpy down under the spinning daemon thread, which
    # aborted with 'terminate called without an active exception'.
    import os, sys
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
