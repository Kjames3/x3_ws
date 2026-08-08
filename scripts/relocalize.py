#!/usr/bin/env python3
"""relocalize.py — global relocalization for AMCL, with convergence monitoring.

Spreads AMCL's particles uniformly over the map's free space, then watches the
pose covariance while you drive. In a small feature-rich space this converges in
a few metres of travel and removes any dependence on knowing the start pose.

Use this instead of trusting the set_initial_pose seed: seeding a WRONG pose is
worse than useless, because AMCL then reports a confident but false position.

Usage (on the robot, with x3_localization already running):
    source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=42
    python3 scripts/relocalize.py [seconds]

Then DRIVE THE ROBOT — preferably through varied geometry (past a doorway, along
a wall, into another room). Rotating on the spot converges yaw but not position.

Reads out 1-sigma uncertainty in x, y and yaw. Converged looks like sub-0.2 m
and sub-10 deg, holding steady. If it stays large or keeps jumping, the filter
has not locked on and the pose should not be trusted.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
CONVERGED_XY = 0.20     # metres, 1-sigma
CONVERGED_YAW = 10.0    # degrees, 1-sigma


class Relocalizer(Node):
    def __init__(self):
        super().__init__("relocalize")
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.cb, 10)
        self.cli = self.create_client(Empty, "/reinitialize_global_localization")
        self.samples = []

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        c = m.pose.covariance
        self.samples.append((
            time.time(), p.x, p.y, math.degrees(yaw),
            math.sqrt(max(c[0], 0.0)),
            math.sqrt(max(c[7], 0.0)),
            math.degrees(math.sqrt(max(c[35], 0.0))),
        ))


def main():
    rclpy.init()
    n = Relocalizer()

    print("waiting for /reinitialize_global_localization ...")
    if not n.cli.wait_for_service(timeout_sec=10.0):
        print("SERVICE NOT AVAILABLE - is x3_localization.launch.py running?")
        sys.exit(2)

    fut = n.cli.call_async(Empty.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10.0)
    print("particles scattered over the map's free space.\n")
    print("DRIVE THE ROBOT NOW - through varied geometry, not just spinning.\n")
    print(f"{'t':>5} | {'x':>7} {'y':>7} {'yaw':>7} | "
          f"{'sig_x':>6} {'sig_y':>6} {'sig_yaw':>8} | state")

    t0 = time.time()
    last = 0.0
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.time()
        if now - last >= 2.0 and n.samples:
            last = now
            _, x, y, yaw, sx, sy, syaw = n.samples[-1]
            conv = (sx < CONVERGED_XY and sy < CONVERGED_XY
                    and syaw < CONVERGED_YAW)
            print(f"{now-t0:5.1f} | {x:7.3f} {y:7.3f} {yaw:7.1f} | "
                  f"{sx:6.3f} {sy:6.3f} {syaw:8.1f} | "
                  f"{'CONVERGED' if conv else 'searching'}")

    if not n.samples:
        print("\nNO /amcl_pose RECEIVED - AMCL only updates after the robot moves "
              "past update_min_d (0.25 m) / update_min_a (0.2 rad). Did it move?")
        sys.exit(2)

    _, x, y, yaw, sx, sy, syaw = n.samples[-1]
    print(f"\nfinal pose : x={x:+.3f} m  y={y:+.3f} m  yaw={yaw:+.1f} deg")
    print(f"1-sigma    : x={sx:.3f} m  y={sy:.3f} m  yaw={syaw:.1f} deg")
    if sx < CONVERGED_XY and sy < CONVERGED_XY and syaw < CONVERGED_YAW:
        print("\nCONVERGED - the pose can be trusted. Save it by leaving AMCL running;")
        print("it will track from here.")
        sys.exit(0)
    print("\nNOT CONVERGED - do not trust this pose. Drive further through varied")
    print("geometry, or set the pose manually with RViz's '2D Pose Estimate'.")
    sys.exit(1)


if __name__ == "__main__":
    main()
