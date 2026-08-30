"""set_amcl_pose.py - seed AMCL with a known pose, reliably.

    python3 scripts/set_amcl_pose.py <x> <y> <yaw_deg>

Why this exists rather than using RViz's "2D Pose Estimate": RViz publishes the
pose in ITS FIXED FRAME. If that is set to anything but "map" - "odom" is the
common trap - AMCL rejects every click with

    Ignoring initial pose in frame "odom"; initial poses must be in the
    global frame, "map"

which appears only in the AMCL log, so from the operator's side the tool just
silently does nothing while the filter keeps wandering.

This publishes in "map" explicitly and waits for AMCL to discover the publisher
first, then reports the resulting pose and covariance so you can confirm it took.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

X = float(sys.argv[1]) if len(sys.argv) > 1 else -0.05
Y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.20
YAW_DEG = float(sys.argv[3]) if len(sys.argv) > 3 else 185.0


class S(Node):
    def __init__(self):
        super().__init__("set_pose")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         "/initialpose", qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.cb, 10)
        self.got = []

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        c = m.pose.covariance
        self.got.append((p.x, p.y, math.degrees(yaw),
                         math.sqrt(max(c[0], 0)), math.sqrt(max(c[7], 0)),
                         math.degrees(math.sqrt(max(c[35], 0)))))


rclpy.init()
n = S()

# Wait for AMCL to discover the publisher. Publishing immediately loses the
# message to a DDS discovery race: /initialpose subscribers are VOLATILE, so a
# message sent before the match is simply dropped, with no error anywhere. This
# cost one confusing debugging round.
for _ in range(50):
    if n.pub.get_subscription_count() > 0:
        break
    rclpy.spin_once(n, timeout_sec=0.1)
if n.pub.get_subscription_count() == 0:
    print("WARNING: no subscriber on /initialpose - is AMCL running?")

m = PoseWithCovarianceStamped()
m.header.frame_id = "map"
m.header.stamp = n.get_clock().now().to_msg()
m.pose.pose.position.x = X
m.pose.pose.position.y = Y
yaw = math.radians(YAW_DEG)
m.pose.pose.orientation.z = math.sin(yaw / 2.0)
m.pose.pose.orientation.w = math.cos(yaw / 2.0)
# Tight-ish covariance: we trust this pose, but leave room to refine.
cov = [0.0] * 36
cov[0] = 0.05      # x
cov[7] = 0.05      # y
cov[35] = 0.03     # yaw
m.pose.covariance = cov

for _ in range(3):
    n.pub.publish(m)
    time.sleep(0.3)
print(f"published /initialpose: x={X:+.3f} y={Y:+.3f} yaw={YAW_DEG:+.1f} deg")

t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)

if n.got:
    x, y, yw, sx, sy, syaw = n.got[-1]
    print(f"AMCL now reports: x={x:+.3f} y={y:+.3f} yaw={yw:+.1f} deg")
    print(f"1-sigma: x={sx:.3f} y={sy:.3f} yaw={syaw:.1f} deg")
else:
    print("no /amcl_pose yet (AMCL only republishes after the robot moves) —")
    print("the seed was still delivered; drive to confirm it holds.")
