"""In-place rotation check: command a yaw rate, stop, compare the real rotation.

Run ON THE ROBOT (needs the workspace sourced).  Compares the scan match, AMCL and
/odom yaw at rest, so the robot's real rotation is what the commanded rate is
judged against.  The file name must contain "base_speed_test.py": the server's
/cmd_vel heartbeat yields to scripts named like that (_STANDALONE_TESTS in
server_x3.py).  Under any other name its zeros dilute your commands and a healthy
base looks like it cannot turn (mistaken for a rotation deadband, 2026-10-21).

    scp scripts/rot_base_speed_test.py x3:/home/jetson/rot_base_speed_test.py
    ssh x3 'cd ~/x3_ws/src && source /opt/ros/humble/setup.bash && \\
            source ../install/setup.bash && ROS_DOMAIN_ID=42 \\
            python3 /home/jetson/rot_base_speed_test.py'
"""
import sys, time, math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import Buffer, TransformListener
sys.path.insert(0,"/home/jetson/x3_ws/src")
from auto_initial_pose import LikelihoodField, ScanMatcher, scan_to_points
rclpy.init(); n=Node("rot"); pub=n.create_publisher(Twist,"/cmd_vel",10)
S={"scan":None,"odom":None,"gyro":0.0,"gz":[]}
n.create_subscription(LaserScan,"/scan",lambda m:S.__setitem__("scan",m),qos_profile_sensor_data)
n.create_subscription(Odometry,"/odom",lambda m:S.__setitem__("odom",m),10)
buf=Buffer(); TransformListener(buf,n)
sm=ScanMatcher(LikelihoodField.from_yaml("/home/jetson/x3_ws/src/yahboomcar_nav/maps/RealLab3.yaml"))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def spin(sec):
    t=time.time()
    while time.time()-t<sec: rclpy.spin_once(n,timeout_sec=0.05)
def drive(wz,sec):
    t=time.time()
    while time.time()-t<sec:
        m=Twist(); m.angular.z=wz; pub.publish(m); rclpy.spin_once(n,timeout_sec=0.05)
    for _ in range(15): pub.publish(Twist()); rclpy.spin_once(n,timeout_sec=0.05)
def snap(label):
    spin(3.0); S["scan"]=None; spin(0.6)
    s=S["scan"]; pts=scan_to_points(np.asarray(s.ranges),s.angle_min,s.angle_increment,s.range_min,6.0,0.0435,0.0,math.pi+math.radians(4.85))
    r=sm.locate(pts); tf=buf.lookup_transform("map","base_link",rclpy.time.Time())
    a=(tf.transform.translation.x,tf.transform.translation.y,math.degrees(yaw(tf.transform.rotation)))
    o=S["odom"].pose.pose; od=(o.position.x,o.position.y,math.degrees(yaw(o.orientation)))
    d=lambda x:((x+180)%360)-180
    print("%-22s match (%.2f,%.2f,%6.1f)  amcl (%.2f,%.2f,%6.1f)  odom yaw %6.1f   amcl-match Δyaw %5.1f° Δpos %.2f m"%(label,r.x,r.y,math.degrees(r.yaw),a[0],a[1],a[2],od[2],d(a[2]-math.degrees(r.yaw)),math.hypot(a[0]-r.x,a[1]-r.y)),flush=True)
    return math.degrees(r.yaw),a[2],od[2]
spin(2)
prev=snap("start")
for wz,sec in ((0.5,3.0),(-0.5,3.0),(0.3,5.0),(-0.3,5.0)):
    drive(wz,sec); cur=snap("after wz=%+.1f x %.0fs"%(wz,sec))
    d=lambda x:((x+180)%360)-180
    print("     rotated: match %+6.1f°  amcl %+6.1f°  odom %+6.1f°   (commanded %+6.1f°)"%(d(cur[0]-prev[0]),d(cur[1]-prev[1]),d(cur[2]-prev[2]),math.degrees(wz*sec)),flush=True)
    prev=cur
