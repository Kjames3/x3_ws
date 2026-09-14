"""Opt-in diagnostic localizer: no TF broadcaster and no velocity publisher.

Input is deskewed PointCloud2 in its scan-start sensor frame. TF at that same
stamp places it in base_footprint; odom supplies the tracking prior. Initialize
with PoseStamped on ~/initial_pose in the map file's frame. Rejected matches
publish status only and never update map-to-odom correction or pose output.
"""
import json,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation
from .lidar_registration import Matcher,pose_matrix,transform
from .lidar_tracking import Tracker


def matrix(t):
    q=t.rotation;p=t.translation
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[p.x,p.y,p.z]
    return T


def xyz(msg):
    fields={f.name:f for f in msg.fields}
    if any(k not in fields or fields[k].datatype!=7 or fields[k].count!=1 for k in ('x','y','z')):
        raise ValueError('requires scalar float32 XYZ')
    dtype=np.dtype({'names':['x','y','z'],'formats':[('>' if msg.is_bigendian else '<')+'f4']*3,
                    'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':msg.point_step})
    a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
    return np.column_stack([a[k].ravel() for k in ('x','y','z')])


class LidarLocalizer(Node):
    def __init__(self):
        super().__init__('lidar_localizer')
        self.declare_parameter('map_path','')
        self.declare_parameter('cloud_topic','/pointcloud_raw')
        self.declare_parameter('base_frame','base_footprint')
        self.declare_parameter('odom_frame','odom')
        self.declare_parameter('max_age_s',.5)
        path=self.get_parameter('map_path').value
        with np.load(path,allow_pickle=False) as z:
            self.matcher=Matcher(z['points'],z['normals']);self.map_frame=str(z['frame'].item())
        self.base=self.get_parameter('base_frame').value;self.odom=self.get_parameter('odom_frame').value
        self.max_age=float(self.get_parameter('max_age_s').value)
        if not 0.<self.max_age<=.5:raise ValueError('max_age_s must be in (0,0.5]')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pose_pub=self.create_publisher(PoseStamped,'~/pose',10)
        self.status_pub=self.create_publisher(String,'~/status',10)
        self.tracker=Tracker(self.matcher)
        self.pending=None;self.last_stamp=0;self.last_rx=time.monotonic();self.replaced=0
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub=self.create_subscription(PointCloud2,self.get_parameter('cloud_topic').value,self.receive,qos)
        self.init_sub=self.create_subscription(PoseStamped,'~/initial_pose',self.initialize,10)
        self.pause_sub=self.create_subscription(Bool,'~/paused',self.pause,10)
        self.timer=self.create_timer(.02,self.process)
        self.watchdog=self.create_timer(1.,self.check_input)
        self.status('awaiting_initial_pose')

    def status(self,state,**details):
        def clean(v):
            if isinstance(v,float) and not np.isfinite(v):return None
            if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
            return v
        self.status_pub.publish(String(data=json.dumps(clean(dict(state=state,localization_state=self.tracker.state,replaced=self.replaced,**details)),allow_nan=False)))

    def odom_pose(self,stamp):
        t=matrix(self.buffer.lookup_transform(self.odom,self.base,stamp).transform)
        # This is a flat-floor estimator; withhold if that model is violated.
        if t[2,2]<np.cos(np.deg2rad(5)):
            raise ValueError('base is not upright')
        return pose_matrix(t[0,3],t[1,3],np.arctan2(t[1,0],t[0,0]))

    def initialize(self,msg):
        try:
            if msg.header.frame_id!=self.map_frame:raise ValueError('initial pose frame does not match map')
            stamp=Time.from_msg(msg.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds*1e-9
            if not 0<=age<=self.max_age:raise ValueError('initial pose must have a recent nonzero stamp')
            q=msg.pose.orientation;p=msg.pose.position
            R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            if not np.isfinite([p.x,p.y,p.z]).all() or abs(p.z)>.05 or R[2,2]<np.cos(np.deg2rad(5)):
                raise ValueError('initial pose must be finite and planar')
            initial=pose_matrix(p.x,p.y,np.arctan2(R[1,0],R[0,0]))
            self.tracker.initialize(initial@np.linalg.inv(self.odom_pose(stamp)),stamp.nanoseconds*1e-9)
            self.pending=None;self.last_stamp=stamp.nanoseconds
            self.status('initialized')
        except Exception as e:self.status('initialization_rejected',reason=str(e))

    @property
    def correction(self):
        return self.tracker.correction

    def pause(self,msg):
        self.pending=None
        self.tracker.pause(bool(msg.data))
        self.status('paused' if msg.data else 'resumed')

    def receive(self,msg):
        if self.pending is not None:self.replaced+=1
        self.pending=msg;self.last_rx=time.monotonic()

    def check_input(self):
        if time.monotonic()-self.last_rx>1.:
            self.tracker.expire(self.get_clock().now().nanoseconds*1e-9)
            self.status('paused' if self.tracker.state=='PAUSED' else 'no_recent_clouds')

    def process(self):
        msg=self.pending
        if msg is None:return
        self.pending=None
        if self.tracker.state=='PAUSED':self.status('paused');return
        if self.correction is None:self.status('awaiting_initial_pose');return
        stamp=Time.from_msg(msg.header.stamp)
        age=(self.get_clock().now()-stamp).nanoseconds*1e-9
        if stamp.nanoseconds<=self.last_stamp or not 0<=age<=self.max_age:
            self.tracker.reject_input(self.get_clock().now().nanoseconds*1e-9,'stale_or_out_of_order')
            self.status('rejected',reasons=['stale_or_out_of_order'],age_s=age);return
        self.last_stamp=stamp.nanoseconds
        try:
            odom=self.odom_pose(stamp)
            sensor=matrix(self.buffer.lookup_transform(self.base,msg.header.frame_id,stamp).transform)
            points=transform(xyz(msg),sensor)
            sensor_pitch=float(np.arctan2(-sensor[2,0],np.hypot(sensor[0,0],sensor[1,0])))
            previous=self.correction.copy();previous_trusted=self.tracker.last_trusted
            candidate,metrics=self.tracker.process(points,odom,stamp.nanoseconds*1e-9,
                sensor_pitch,now=self.get_clock().now().nanoseconds*1e-9)
            metrics.pop('state')  # status carries the explicit localization_state
            age=(self.get_clock().now()-stamp).nanoseconds*1e-9
            if age>self.max_age:
                # A late result must not update even the internal trusted pose.
                self.tracker.correction=previous;self.tracker.last_trusted=previous_trusted
                self.tracker._clear();self.tracker.state='LOST'
                self.status('rejected',reasons=['stale_result'],age_s=age);return
            if candidate is None:
                self.status('withheld',age_s=age,**metrics);return
            out=PoseStamped();out.header.stamp=msg.header.stamp;out.header.frame_id=self.map_frame
            out.pose.position.x=float(candidate[0,3]);out.pose.position.y=float(candidate[1,3])
            yaw=np.arctan2(candidate[1,0],candidate[0,0]);out.pose.orientation.z=float(np.sin(yaw/2));out.pose.orientation.w=float(np.cos(yaw/2))
            self.pose_pub.publish(out);self.status('accepted',age_s=age,**metrics)
        except Exception as e:
            self.tracker.reject_input(self.get_clock().now().nanoseconds*1e-9,'input_or_tf_error')
            self.status('rejected',reasons=['input_or_tf_error'],detail=str(e))


def main(args=None):
    rclpy.init(args=args);node=None
    try:
        node=LidarLocalizer();rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__':main()
