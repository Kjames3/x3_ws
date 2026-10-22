"""Nav2 stall recorder: JSONL at ~10 Hz while a goal runs.  Run ON THE ROBOT.

    cd ~/x3_ws/src && source /opt/ros/humble/setup.bash && source ../install/setup.bash
    ROS_DOMAIN_ID=42 python3 scripts/nav_stall_recorder.py <seconds> <out.jsonl> [map.yaml]

Per row: /cmd_vel_nav (MPPI), /cmd_vel (after the smoother) with message age, /odom
and /odom_raw velocity, /wheel_encoders (wheel speeds, PWM, ramped targets), the
driver's own delivery counters from /driver_diag (cmd_cb per second, longest gap,
watchdog trips, executor tick gap), the map->base_link pose, a scan match against
the map every 4 s (a scan taken while turning is skewed: trust it at rest), and
costmap stats around the robot.  Written 2026-10-21/22 to separate "MPPI asked for
nothing" from "the driver never received it" from "the wheels ignored it".
Run it from a file whose name does not matter: it only subscribes.
"""
import sys, time, json, math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, JointState
from tf2_ros import Buffer, TransformListener
sys.path.insert(0, "/home/jetson/x3_ws/src")
from auto_initial_pose import LikelihoodField, ScanMatcher, scan_to_points
dur=float(sys.argv[1]); out=open(sys.argv[2],"w")
rclpy.init(); n=Node("rec2"); t0=time.time()
last={"diag":None,"enc":None,"nav":None,"cmd":None,"odom":None,"raw":None,"scan":None,"lc":None}
n.create_subscription(Twist,"/cmd_vel_nav",lambda m:last.__setitem__("nav",(m.linear.x,m.linear.y,m.angular.z,time.time())),10)
n.create_subscription(Twist,"/cmd_vel",lambda m:last.__setitem__("cmd",(m.linear.x,m.linear.y,m.angular.z,time.time())),10)
n.create_subscription(Odometry,"/odom",lambda m:last.__setitem__("odom",(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)),10)
n.create_subscription(Odometry,"/odom_raw",lambda m:last.__setitem__("raw",(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)),10)
n.create_subscription(LaserScan,"/scan",lambda m:last.__setitem__("scan",m),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,"/local_costmap/costmap",lambda m:last.__setitem__("lc",m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(JointState,"/wheel_encoders",lambda m:last.__setitem__("enc",([round(x,3) for x in m.velocity],[round(x) for x in m.effort[:4]],[round(x,3) for x in m.effort[4:]])),50)
n.create_subscription(String,"/driver_diag",lambda m:last.__setitem__("diag",json.loads(m.data)),10)
buf=Buffer(); TransformListener(buf,n)
sm=ScanMatcher(LikelihoodField.from_yaml(sys.argv[3] if len(sys.argv)>3 else "/home/jetson/x3_ws/src/yahboomcar_nav/maps/RealLab3.yaml"))
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def costmap_stats(m,px,py):
    if m is None: return None
    a=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width); res=m.info.resolution
    ys,xs=np.nonzero(a>=99)
    if len(xs)==0: return {"lethal":0}
    wx=m.info.origin.position.x+(xs+.5)*res; wy=m.info.origin.position.y+(ys+.5)*res
    d=np.hypot(wx-px,wy-py)
    return {"lethal":int(len(xs)),"nearest_lethal_m":round(float(d.min()),3),"within_0.35":int((d<0.35).sum()),"max_cost_here":int(a[int((py-m.info.origin.position.y)/res),int((px-m.info.origin.position.x)/res)]) if 0<=int((py-m.info.origin.position.y)/res)<m.info.height and 0<=int((px-m.info.origin.position.x)/res)<m.info.width else None}
next_match=0
while time.time()-t0<dur:
    rclpy.spin_once(n,timeout_sec=0.1)
    now=time.time(); rec={"t":round(now-t0,2)}
    for k in ("nav","cmd"):
        v=last[k]; rec[k]=None if v is None else [round(v[0],3),round(v[1],3),round(v[2],3),round(now-v[3],2)]
    rec["enc"]=last["enc"]
    d=last["diag"]
    rec["diag"]=None if d is None else {k:d[k] for k in ("cmd_cb","cmd_cb_nz","cmd_gap_max_ms","cmd_age_ms","pubs","tick_gap_max_ms","wd_trips","tgt_max","pwm_max")}
    for k in ("odom","raw"):
        v=last[k]; rec[k]=None if v is None else [round(x,3) for x in v]
    try:
        tf=buf.lookup_transform("map","base_link",rclpy.time.Time())
        px,py=tf.transform.translation.x,tf.transform.translation.y; rec["amcl"]=[round(px,3),round(py,3),round(math.degrees(yaw_of(tf.transform.rotation)),1)]
        if int(now*10)%20==0 and last["lc"] is not None:
            # The local costmap is a rolling window in ITS OWN frame (odom), not map:
            # use the robot pose in that frame or every distance is off by map->odom.
            cf=last["lc"].header.frame_id
            ct=buf.lookup_transform(cf,"base_link",rclpy.time.Time())
            rec["lc"]=costmap_stats(last["lc"],ct.transform.translation.x,ct.transform.translation.y)
    except Exception as e: rec["amcl"]=None
    if now>=next_match and last["scan"] is not None:
        next_match=now+4; s=last["scan"]
        pts=scan_to_points(np.asarray(s.ranges),s.angle_min,s.angle_increment,s.range_min,6.0,0.0435,0.0,math.pi+math.radians(4.85))
        r=sm.locate(pts); rec["match"]=[round(r.x,3),round(r.y,3),round(math.degrees(r.yaw),1),round(r.score,3)]
    out.write(json.dumps(rec)+"\n"); out.flush(); time.sleep(0.05)
out.write(json.dumps({"done":True})+"\n"); out.close()
