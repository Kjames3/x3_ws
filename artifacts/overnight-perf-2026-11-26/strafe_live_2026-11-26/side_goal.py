#!/usr/bin/env python3
"""Send a NavigateToPose goal offset in the robot frame, keeping current map yaw.
usage: side_goal.py DX DY [yaw_mode: keep|zero]   (DY>0 = left)"""
import math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
from types import SimpleNamespace as NS

dx, dy = float(sys.argv[1]), float(sys.argv[2])
mode = sys.argv[3] if len(sys.argv) > 3 else 'keep'
rclpy.init(); n = Node('strafe_test_goal')
pose = {}; cmds = []; odo = []
buf = tf2_ros.Buffer(); lst = tf2_ros.TransformListener(buf, n)
def getpose():
    try:
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        pose['p'] = NS(position=t.translation, orientation=t.rotation)
    except Exception: pass
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
t0 = [None]
n.create_subscription(Twist, '/cmd_vel_nav', lambda m: cmds.append((time.time(), 'nav', m.linear.x, m.linear.y, m.angular.z)), 50)
n.create_subscription(Twist, '/cmd_vel_safe', lambda m: cmds.append((time.time(), 'safe', m.linear.x, m.linear.y, m.angular.z)), 50)
n.create_subscription(Odometry, '/odom', lambda m: odo.append((time.time(), yaw_of(m.pose.pose.orientation))), 50)
end = time.time() + 5
while 'p' not in pose and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1); getpose()
if 'p' not in pose: print('no map->base_footprint TF'); sys.exit(1)
p = pose['p']; yaw = yaw_of(p.orientation)
gx = p.position.x + dx*math.cos(yaw) - dy*math.sin(yaw)
gy = p.position.y + dx*math.sin(yaw) + dy*math.cos(yaw)
gyaw = yaw if mode == 'keep' else 0.0
print(f'start ({p.position.x:.2f},{p.position.y:.2f}) yaw {math.degrees(yaw):.0f}  goal ({gx:.2f},{gy:.2f}) yaw {math.degrees(gyaw):.0f}')
ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); ac.wait_for_server(timeout_sec=5)
g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.header.stamp = n.get_clock().now().to_msg()
g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw/2), math.cos(gyaw/2)
ts = time.time(); fut = ac.send_goal_async(g)
while not fut.done(): rclpy.spin_once(n, timeout_sec=0.05)
gh = fut.result()
if not gh.accepted: print('rejected'); sys.exit(1)
rf = gh.get_result_async()
while not rf.done() and time.time() - ts < 45: rclpy.spin_once(n, timeout_sec=0.05)
el = time.time() - ts
status = rf.result().status if rf.done() else 'TIMEOUT'
if not rf.done(): gh.cancel_goal_async(); [rclpy.spin_once(n, timeout_sec=0.1) for _ in range(10)]
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
getpose()
q = pose['p']; fy = yaw_of(q.orientation)
err = math.hypot(q.position.x-gx, q.position.y-gy)
print(f'status {status} (4=SUCCEEDED,6=ABORTED) in {el:.1f}s, pos err {err:.2f} m, final yaw {math.degrees(fy):.0f}, heading change {math.degrees(math.atan2(math.sin(fy-yaw),math.cos(fy-yaw))):+.0f} deg')
oy = [y for t,y in odo]
if oy: print(f'max odom heading excursion {max(abs(math.degrees(math.atan2(math.sin(y-oy[0]),math.cos(y-oy[0])))) for y in oy):.0f} deg')
for src in ('nav', 'safe'):
    c = [r for r in cmds if r[1] == src]
    if not c: print(src, 'no msgs'); continue
    first_vy = next((r[0]-ts for r in c if abs(r[3]) > 0.15), None)
    print(f'/cmd_vel_{src}: n={len(c)} max|vx| {max(abs(r[2]) for r in c):.2f} max|vy| {max(abs(r[3]) for r in c):.2f} max|wz| {max(abs(r[4]) for r in c):.2f}; first |vy|>0.15 at {first_vy if first_vy is None else round(first_vy,1)} s; '
          f'frac |wz|>0.5: {sum(abs(r[4])>0.5 for r in c)/len(c):.0%}')
    print('  first 2 s (t, vx, vy, wz):', [(round(r[0]-ts,1), round(r[2],2), round(r[3],2), round(r[4],2)) for r in c if r[0]-ts < 2][::5])
