import time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters,GetParameters
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan,PointCloud,PointCloud2
from geometry_msgs.msg import Twist
rclpy.init(); n=Node('deploy_verify'); counts={}; max_range=0.; max_cmd=0.; channels=[]
def cb(msg,key):
 global max_range,max_cmd,channels
 counts[key]=counts.get(key,0)+1
 if key=='cloud' and msg.width:
  p=np.frombuffer(msg.data,dtype='<f4').reshape(-1,msg.point_step//4)[:,:3]
  max_range=max(max_range,float(np.linalg.norm(p,axis=1).max()))
 if key=='cmd':max_cmd=max(max_cmd,abs(msg.linear.x),abs(msg.linear.y),abs(msg.angular.z))
 if key=='timed':channels=[c.name for c in msg.channels]
subs=[n.create_subscription(t,topic,lambda m,k=k:cb(m,k),qos_profile_sensor_data) for t,topic,k in [(LaserScan,'/scan','scan'),(PointCloud,'/lidar/points_timed','timed'),(PointCloud2,'/pointcloud_raw','cloud'),(Twist,'/cmd_vel','cmd')]]
def call(service,request,typ):
 c=n.create_client(typ,service)
 if not c.wait_for_service(timeout_sec=15):raise RuntimeError(service)
 f=c.call_async(request); rclpy.spin_until_future_complete(n,f,timeout_sec=10)
 if not f.done():raise RuntimeError('timeout')
 return f.result()
def setp(name,value):
 r=call('/lidar_3d_processor_node/set_parameters',SetParameters.Request(parameters=[Parameter(name,value=value).to_parameter_msg()]),SetParameters)
 assert r.results[0].successful,r

def sample(label):
 global counts,max_range
 counts={};max_range=0.
 end=time.monotonic()+8
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 result={'phase':label,'counts':counts.copy(),'max_point_range':max_range,'max_cmd':max_cmd,'channels':channels}
 print(json.dumps(result),flush=True)
 assert counts.get('scan',0)>20 and counts.get('timed',0)>20,result
 assert max_cmd==0,result
 return result
r=call('/lidar_3d_processor_node/get_parameters',GetParameters.Request(names=['cloud_max_range_m','require_settled','publish_cloud_when_level']),GetParameters)
print(str(r),flush=True)
assert r.values[0].double_value==6. and r.values[1].bool_value and not r.values[2].bool_value
try:
 sample('baseline')
 setp('publish_cloud_when_level',True)
 setp('cloud_max_range_m',1.0)
 sample('settle')
 r=sample('1m');assert r['counts'].get('cloud',0)>20 and r['max_point_range']<1.01
 setp('cloud_max_range_m',6.0)
 r=sample('6m');assert r['counts'].get('cloud',0)>20 and 1.<r['max_point_range']<6.01
finally:
 setp('cloud_max_range_m',6.0);setp('publish_cloud_when_level',False)
sample('restored')
n.destroy_node();rclpy.shutdown()
