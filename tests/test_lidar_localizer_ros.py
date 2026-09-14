import sys,json
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped,PoseStamped
from sensor_msgs.msg import PointCloud2,PointField
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/yahboomcar_bringup'))
from yahboomcar_bringup.lidar_localizer_node import LidarLocalizer,xyz
from yahboomcar_bringup.lidar_registration import pose_matrix

@pytest.fixture
def node(tmp_path):
    rng=np.random.default_rng(42);p=rng.uniform(-2,2,(1200,3));p[:400,0]=2;p[400:800,1]=2;p[800:,2]=0
    n=np.zeros_like(p);n[:400,0]=1;n[400:800,1]=1;n[800:,2]=1
    path=tmp_path/'map.npz';np.savez(path,points=p,normals=n,frame=np.array('lidar_map'))
    rclpy.init(args=['--ros-args','-p',f'map_path:={path}'])
    node=LidarLocalizer();poses=[];statuses=[]
    node.pose_pub=SimpleNamespace(publish=poses.append)
    node.status_pub=SimpleNamespace(publish=lambda m:statuses.append(json.loads(m.data)))
    node.matcher.limits=replace(node.matcher.limits,max_runtime_s=2.)
    for parent,child in [('odom','base_footprint'),('base_footprint','test_laser')]:
        tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child;tf.transform.rotation.w=1.
        node.buffer.set_transform_static(tf,'test')
    msg=PointCloud2();msg.header.frame_id='test_laser';msg.header.stamp=Time(nanoseconds=node.get_clock().now().nanoseconds-20_000_000).to_msg()
    msg.height=1;msg.width=len(p);msg.point_step=12;msg.row_step=12*len(p)
    msg.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])];msg.data=p.astype('<f4').tobytes()
    yield node,msg,poses,statuses
    node.destroy_node();rclpy.shutdown()


def feed_consensus(n,m):
    from scipy.spatial.transform import Rotation
    p=xyz(m).copy();start=n.get_clock().now().nanoseconds-470_000_000
    n.tracker.initialize(pose_matrix(.05,-.04,.01),start*1e-9-.01)
    for i in range(4):
        pitch=.07*i
        tf=TransformStamped();tf.header.frame_id='base_footprint';tf.child_frame_id='test_laser'
        tf.transform.rotation.y=float(np.sin(pitch/2));tf.transform.rotation.w=float(np.cos(pitch/2))
        n.buffer.set_transform_static(tf,'test')
        m.header.stamp=Time(nanoseconds=start+i*120_000_000).to_msg()
        m.data=(p@Rotation.from_euler('y',pitch).as_matrix()).astype('<f4').tobytes()
        n.receive(m);n.process()


def test_no_pose_before_explicit_initialization(node):
    n,m,p,s=node;n.receive(m);n.process()
    assert not p and s[-1]['state']=='awaiting_initial_pose'


def test_accept_then_reject_does_not_update_correction(node):
    n,m,p,s=node;feed_consensus(n,m)
    assert len(p)==1 and s[-1]['state']=='accepted',s
    assert p[0].header.stamp==m.header.stamp and p[0].header.frame_id=='lidar_map'
    correction=n.correction.copy();n.receive(m);n.process()
    assert len(p)==1 and s[-1]['reasons']==['stale_or_out_of_order']
    np.testing.assert_array_equal(correction,n.correction)
    m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='missing';n.receive(m);n.process()
    assert len(p)==1 and s[-1]['reasons']==['input_or_tf_error']
    np.testing.assert_array_equal(correction,n.correction)


def test_stale_cloud_cannot_publish(node):
    n,m,p,s=node;n.tracker.initialize(pose_matrix(),0.);m.header.stamp=Time(nanoseconds=n.get_clock().now().nanoseconds-2_000_000_000).to_msg()
    n.receive(m);n.process();assert not p and s[-1]['reasons']==['stale_or_out_of_order']


def test_initialization_checks_frame_and_timestamp(node):
    n,m,p,s=node;initial=PoseStamped();initial.header.stamp=n.get_clock().now().to_msg();initial.header.frame_id='wrong';initial.pose.orientation.w=1.
    n.initialize(initial);assert n.correction is None
    initial.header.frame_id='lidar_map';n.initialize(initial)
    np.testing.assert_allclose(n.correction,np.eye(4));assert s[-1]['state']=='initialized'


def test_latest_only_pending_queue(node):
    n,m,p,s=node;n.receive(m);n.receive(m);assert n.replaced==1 and n.pending is m


def test_cloud_decoding_handles_padding_and_endianness():
    m=PointCloud2();m.height=2;m.width=1;m.point_step=16;m.row_step=20;m.is_bigendian=True
    m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z'])]
    m.data=np.array([1,2,3,0,0,4,5,6,0,0],'>f4').tobytes()
    np.testing.assert_array_equal(xyz(m),[[1,2,3],[4,5,6]])


def test_pause_is_explicit_and_does_not_refresh_pose(node):
    from std_msgs.msg import Bool
    n,m,p,s=node
    n.tracker.initialize(pose_matrix(),0.)
    n.pause(Bool(data=True));n.receive(m);n.process();n.check_input()
    assert not p and n.tracker.state=='PAUSED'
    n.pause(Bool(data=False));assert n.tracker.state=='SUSPECT'


def test_stale_completed_result_cannot_update_internal_correction(node):
    n,m,p,s=node
    n.tracker.initialize(pose_matrix(),0.)
    original=n.tracker.correction.copy()
    def late(*args,**kwargs):
        n.tracker.correction=pose_matrix(1.)
        n.max_age=0.  # Simulate the deadline expiring during matching.
        return pose_matrix(1.),dict(state='TRACKING',accepted=True,reason='temporal_consensus')
    n.tracker.process=late;n.receive(m);n.process()
    assert not p and n.tracker.state=='LOST'
    np.testing.assert_array_equal(n.tracker.correction,original)
