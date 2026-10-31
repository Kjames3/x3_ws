"""Subscriber-gated, display-only OAK colored cloud. Never feeds safety topics."""
import array
import json
import logging
import threading
import time

import cv2
import numpy as np
from oak_color_geometry import refine

log = logging.getLogger(__name__)


def transform_xyz(points, transform):
    q = transform.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    t=transform.translation
    # Elementwise avoids multithreaded BLAS overhead on tiny 3-column arrays.
    return (points[:,0,None]*rotation[:,0]+points[:,1,None]*rotation[:,1]+
            points[:,2,None]*rotation[:,2]+[t.x,t.y,t.z]).astype('<f4')


class OakSurroundings:
    def __init__(self,node,camera):
        from sensor_msgs.msg import PointCloud2,PointField
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer,TransformListener
        from rclpy.duration import Duration
        self.node,self.camera=node,camera
        self.Cloud,self.Field,self.Pose=PointCloud2,PointField,PoseStamped
        self.pub=node.create_publisher(PointCloud2,'/surroundings/oak/points',qos_profile_sensor_data)
        self.pose_pub=node.create_publisher(PoseStamped,'/surroundings/oak/pose',qos_profile_sensor_data)
        self.tf=Buffer(cache_time=Duration(seconds=3))
        self.Listener=TransformListener
        self.listener=None
        self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,name='oak-surroundings',daemon=True)
        self.last=None
        self.maps_key=None
        self.maps=None
        self.offset=None

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set();self.thread.join(timeout=3)
        if self.listener is not None: self.listener.unregister()

    def run(self):
        stats=[];skipped=0;last_log=time.monotonic();last_error=0
        while not self.stop_event.is_set():
            start=time.monotonic()
            cpu_start=time.thread_time()
            if self.pub.get_subscription_count():
                if self.listener is None:
                    self.listener=self.Listener(self.tf,self.node)
                try:
                    age=self.publish()
                    if age is not None: stats.append(((time.monotonic()-start)*1000,age,(time.thread_time()-cpu_start)*1000))
                    else: skipped+=1
                except Exception as exc:
                    skipped+=1
                    if start-last_error>10:
                        log.warning('OAK display skipped: %s',exc);last_error=start
            elif self.listener is not None:
                self.listener.unregister();self.listener=None;self.tf.clear()
            if start-last_log>=30:
                if stats:
                    log.info('OAK display: n=%d work_p95=%.1fms age_p95=%.1fms cpu_p95=%.1fms skipped=%d',len(stats),np.percentile(stats,95,axis=0)[0],np.percentile(stats,95,axis=0)[1],np.percentile(stats,95,axis=0)[2],skipped)
                stats=[];skipped=0;last_log=start
            self.stop_event.wait(max(.01,1/3-(time.monotonic()-start)))

    def publish(self):
        from rclpy.time import Time
        pair=self.camera.get_display_rgbd_pair()
        if pair is None: return
        identity=(pair['rgb']['meta']['session_id'],pair['epoch'],pair['pair_seq'])
        if identity==self.last:return
        c=pair['calibration']
        if not c or c.get('depth_recorded')!='raw_uncorrected':return
        source=pair['depth']['meta']['host_monotonic_estimate_ns']
        mono=time.monotonic_ns();age=(mono-source)/1e6
        if not 0<=age<=500:return
        now=self.node.get_clock().now().nanoseconds
        offset=now-mono
        if self.offset is not None and abs(offset-self.offset)>100_000_000:
            self.offset=offset;self.last=identity;return
        self.offset=offset
        stamp=source+offset
        capture=self.tf.lookup_transform('odom','oak_rgb_camera_optical_frame',Time(nanoseconds=stamp))
        pose_tf=self.tf.lookup_transform('odom','base_footprint',Time())
        pose_ns=pose_tf.header.stamp.sec*1_000_000_000+pose_tf.header.stamp.nanosec
        if not 0<=now-pose_ns<=500_000_000:return
        rgb=pair['rgb']['image'];depth=pair['depth']['image']
        if rgb.shape!=(*depth.shape,3) or depth.shape!=(c['height'],c['width']):return
        key=json.dumps({k:c[k] for k in ('rgb_k','rgb_d','depth_k','width','height')},sort_keys=True)
        if key!=self.maps_key:
            k=np.asarray(c['depth_k']).reshape(3,3)
            maps=cv2.initUndistortRectifyMap(np.asarray(c['rgb_k']).reshape(3,3),np.asarray(c['rgb_d']),np.eye(3),k,(c['width'],c['height']),cv2.CV_32FC1)
            support=cv2.remap(np.ones(depth.shape,np.float32),*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)>=1
            self.maps=(maps,support,k);self.maps_key=key
        maps,support,k=self.maps
        aligned=cv2.remap(rgb,*maps,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        xyz,colors,confidence,_=refine(aligned,depth,support,k,c['depth_inv_offset_per_m'])
        xyz=transform_xyz(xyz,capture.transform)
        data=np.empty(len(xyz),dtype=[('xyz','<f4',(3,)),('rgb','<u4')])
        data['xyz']=xyz
        data['rgb']=(colors[:,0].astype(np.uint32)<<16)|(colors[:,1].astype(np.uint32)<<8)|colors[:,2]
        msg=self.Cloud();msg.header.frame_id='odom'
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        msg.height=1;msg.width=len(xyz);msg.point_step=16;msg.row_step=len(xyz)*16
        msg.fields=[self.Field(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]+[self.Field(name='rgb',offset=12,datatype=6,count=1)]
        msg.is_bigendian=False;msg.is_dense=True;msg.data=array.array('B',data.tobytes())
        pose=self.Pose();pose.header=pose_tf.header
        t=pose_tf.transform.translation;q=pose_tf.transform.rotation
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=t.x,t.y,t.z
        pose.pose.orientation=q
        # Drop work that became stale while processing.
        if (time.monotonic_ns()-source)>500_000_000:return
        self.pose_pub.publish(pose);self.pub.publish(msg);self.last=identity
        return (time.monotonic_ns()-source)/1e6
