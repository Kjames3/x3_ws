import time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Twist
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
rclpy.init(); n=Node('oak_coverage_probe'); buf=Buffer(); listener=TransformListener(buf,n)
info=None; rows=[]; arrivals=[]; ages=[]; maxcmd=0.; frames=0; errors=[]; extrinsics=None

def ci(m):
 global info
 info=m

def cmd(m):
 global maxcmd
 maxcmd=max(maxcmd,abs(m.linear.x),abs(m.linear.y),abs(m.angular.z))

def depth(m):
 global frames,extrinsics
 frames+=1; now=time.monotonic();arrivals.append(now)
 ages.append(n.get_clock().now().nanoseconds*1e-9-m.header.stamp.sec-m.header.stamp.nanosec*1e-9)
 if info is None:return
 try:t=buf.lookup_transform('base_footprint',m.header.frame_id,rclpy.time.Time()).transform
 except Exception as e:
  errors.append(str(e));return
 q=t.rotation; tr=t.translation; extrinsics=[tr.x,tr.y,tr.z]
 dt=np.dtype('>u2' if m.is_bigendian else '<u2')
 d=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.step,2))[::4,::4].astype(float)/1000
 v,u=np.mgrid[0:m.height:4,0:m.width:4]; k=info.k
 z=d.ravel(); x=((u-k[2])/k[0]*d).ravel();y=((v-k[5])/k[4]*d).ravel()
 valid=np.isfinite(z)&(z>=.3)&(z<=4.)
 p=np.column_stack((x[valid],y[valid],z[valid]))
 p=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(p)+extrinsics
 bearing=np.degrees(np.arctan2(p[:,1],p[:,0])); band=(p[:,2]>=.12)&(p[:,2]<=.4)
 bins=np.arange(-30,31,5)
 hit=np.histogram(bearing[band],bins)[0]>0
 rows.append([float(valid.mean()),float(hit.mean()),int(band.sum()),float(np.mean((z>0)&(z<.3)))])
subs=[n.create_subscription(CameraInfo,'/oak/depth/camera_info',ci,qos_profile_sensor_data),n.create_subscription(Image,'/oak/depth/image_raw',depth,qos_profile_sensor_data),n.create_subscription(Twist,'/cmd_vel',cmd,qos_profile_sensor_data)]
end=time.monotonic()+35
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
def pct(x):return dict(zip(['p5','p50','p95','max'],np.percentile(x,[5,50,95,100]).tolist())) if len(x) else None
out={'frames':frames,'analyzed':len(rows),'delivery_hz':(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0,'delivery_gap_s':pct(np.diff(arrivals)),'publication_to_callback_s':pct(ages),'max_cmd':maxcmd,'camera_xyz_base_footprint':extrinsics,'tf_errors':errors[:2]}
if info:
 out['image_size']=[info.width,info.height];out['horizontal_fov_degrees']=[float(np.degrees(np.arctan(-info.k[2]/info.k[0]))),float(np.degrees(np.arctan((info.width-1-info.k[2])/info.k[0])))]
if rows:
 a=np.array(rows);out['valid_depth_fraction_0_3_to_4m']=pct(a[:,0]);out['forward_5deg_bins_with_height_band_returns']=pct(a[:,1]);out['height_band_points']=pct(a[:,2])
print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
