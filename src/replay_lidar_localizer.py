#!/usr/bin/env python3
"""Exercise installed diagnostic ROS node on held-out clouds, isolated from hardware.

Uses ROS_DOMAIN_ID=43 (required), synthetic current stamps and identity odometry.
The supplied initial map pose comes from the training graph. This tests message,
TF and failure behavior, not live pose accuracy or recorded acquisition latency.
"""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,TransformStamped
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String, Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from scipy.spatial.transform import Rotation
from rclpy.qos import qos_profile_sensor_data

def main():
 if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('Replay requires isolated ROS_DOMAIN_ID=43')
 p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--capture',required=True);p.add_argument('--out',required=True);a=p.parse_args()
 z=np.load(a.capture);m=np.load(a.map);meta=z['cloud_meta'];ids=[int(c) for c,s,*_ in meta if int(s)==0 and int(c) in set(m['heldout_cloud_ids'])][:12]
 rclpy.init();n=Node('localizer_replay');states=[];poses=[]
 sub=n.create_subscription(String,'/lidar_localizer/status',lambda v:states.append(json.loads(v.data)),10)
 ps=n.create_subscription(PoseStamped,'/lidar_localizer/pose',poses.append,10)
 pub=n.create_publisher(PointCloud2,'/replay/cloud',qos_profile_sensor_data);ip=n.create_publisher(PoseStamped,'/lidar_localizer/initial_pose',10)
 dynamic_tf=TransformBroadcaster(n)
 tf=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='odom';t.child_frame_id='base_footprint';t.transform.rotation.w=1.;tf.sendTransform(t)
 proc=subprocess.Popen([sys.executable,'-m','yahboomcar_bringup.lidar_localizer_node','--ros-args','-p',f'map_path:={Path(a.map).resolve()}','-p','cloud_topic:=/replay/cloud'],start_new_session=True,stdout=subprocess.DEVNULL)
 def spin(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if proc.poll() is not None:raise RuntimeError('localizer exited')
   rclpy.spin_once(n,timeout_sec=.02)
 def cloud(c,old=False):
  p=z['points'][z['cloud_id']==c].astype('<f4');v=PointCloud2();v.header.frame_id='replay_laser';v.header.stamp=n.get_clock().now().to_msg()
  if old:v.header.stamp.sec-=5
  original_stamp=meta[meta[:,0]==c,2][0]
  pitch=float(np.interp(original_stamp,z['joints'][:,0],z['joints'][:,1]))
  R=Rotation.from_euler('y',pitch).as_matrix();p=(p@R).astype('<f4')
  tilt=TransformStamped();tilt.header.stamp=v.header.stamp;tilt.header.frame_id='base_footprint';tilt.child_frame_id='replay_laser'
  tilt.transform.rotation.y=float(np.sin(pitch/2));tilt.transform.rotation.w=float(np.cos(pitch/2));dynamic_tf.sendTransform(tilt)
  v.height=1;v.width=len(p);v.point_step=12;v.row_step=12*len(p);v.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])];v.data=p.tobytes();return v
 try:
  deadline=time.monotonic()+15
  while ip.get_subscription_count()<1 or pub.get_subscription_count()<1:
   if time.monotonic()>deadline:raise RuntimeError('discovery timeout')
   spin(.1)
  spin(.5);pub.publish(cloud(ids[0]));spin(.3)
  assert not poses and any(s['state']=='awaiting_initial_pose' for s in states)
  initial=PoseStamped();initial.header.frame_id=str(m['frame'].item());initial.header.stamp=n.get_clock().now().to_msg();p=m['poses'][0]
  initial.pose.position.x=float(p[0]+.08);initial.pose.position.y=float(p[1]-.06);initial.pose.orientation.z=float(np.sin((p[2]+np.deg2rad(2))/2));initial.pose.orientation.w=float(np.cos((p[2]+np.deg2rad(2))/2))
  ip.publish(initial);spin(.3);assert any(s['state']=='initialized' for s in states),states
  stamps=set()
  for c in ids:
   v=cloud(c);stamps.add((v.header.stamp.sec,v.header.stamp.nanosec));spin(.02);pub.publish(v);spin(.12)
  spin(.4);assert poses,states
  before=len(poses);pub.publish(cloud(ids[0],old=True));spin(.3)
  assert len(poses)==before and any('stale_or_out_of_order' in s.get('reasons',[]) for s in states)
  assert all(p.header.frame_id=='lidar_map' and (p.header.stamp.sec,p.header.stamp.nanosec) in stamps for p in poses)
  # Inspect publishers belonging to the localizer, not test/static TF publishers.
  publishers=n.get_publisher_names_and_types_by_node('lidar_localizer','/')
  assert not any(topic in ['/tf','/tf_static','/cmd_vel'] for topic,_ in publishers),publishers
  spin(2.1);assert any(s['state']=='no_recent_clouds' and s['localization_state']=='LOST' for s in states)
  paused=n.create_publisher(Bool,'/lidar_localizer/paused',10);spin(.3);paused.publish(Bool(data=True));spin(.3)
  assert any(s['localization_state']=='PAUSED' for s in states)
  paused.publish(Bool(data=False));spin(.3)
  assert states[-1]['localization_state'] in ['SUSPECT','LOST']
  result={'clouds_sent':len(ids),'poses_received':before,'states':dict(Counter(s['state'] for s in states)),'localizer_publishers':publishers,'status_messages':states,'passed':True,'note':'Isolated DDS replay, current synthetic timestamps and graph-derived initial pose; not an accuracy/latency capture.'}
  Path(a.out).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='status_messages'},indent=2))
 finally:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
