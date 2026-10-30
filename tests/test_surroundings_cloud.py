import ast, pathlib, struct, subprocess, tempfile

import pytest
pytest.importorskip("sensor_msgs.msg")
pytest.importorskip("rclpy.serialization")
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.serialization import serialize_message
from types import SimpleNamespace as NS
def test_publisher_browser_roundtrip():
    path=pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py')
    tree=ast.parse(path.read_text()); cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_surroundings')
    ns=dict(np=np,struct=struct,PointCloud2=PointCloud2,PointField=PointField)
    exec(compile(ast.Module(body=[method],type_ignores=[]),'cloud','exec'),ns)
    messages=[]
    clock=NS(now=lambda:NS(nanoseconds=1_000_000_000))
    obj=NS(get_clock=lambda:clock, cloud_pub=NS(get_subscription_count=lambda:1,publish=messages.append),pose_at=lambda stamps: None,cloud_last_ns=0,cloud_R=np.eye(3),cloud_t=np.array([.0432,0,.31]))
    rng=np.random.default_rng(1); pts=np.zeros(20000,dtype=[('x','f4'),('y','f4'),('z','f4'),('t','u4')]); xyz=rng.uniform(-4,4,(20000,3))
    for i,k in enumerate('xyz'):pts[k]=xyz[:,i]
    pts['x'][0]=float('nan');pts['x'][1]=pts['y'][1]=pts['z'][1]=0
    ns['publish_surroundings'](obj,pts,struct.pack('<I i I',256,123,456))
    m=messages[0];assert 0<m.width<=6000 and m.header.frame_id=='base_footprint'
    assert np.isfinite(np.frombuffer(m.data,dtype='<f4')).all()
    with tempfile.TemporaryDirectory() as directory:
        fixture = pathlib.Path(directory) / 'cloud.cdr'
        fixture.write_bytes(serialize_message(m))
        subprocess.run(['node', 'tests/web/surroundings_cdr_check.cjs', str(fixture)], check=True)
    ns['publish_surroundings'](obj,pts,struct.pack('<I i I',256,123,456));assert len(messages)==1
    print('Publisher: filtering, transform, cap, rate limit and ROS serialization passed;',m.width,'points')
    clock.now = lambda: NS(nanoseconds=2_000_000_000)
    single = pts[:1].copy()
    single['x'], single['y'], single['z'] = 1, 0, 0
    ns['publish_surroundings'](obj, single, struct.pack('<I i I', 256, 123, 456))
    np.testing.assert_allclose(np.frombuffer(messages[-1].data, dtype='<f4'), [1.0432, 0, .31])
    obj.cloud_pub.get_subscription_count = lambda: 0
    clock.now = lambda: NS(nanoseconds=3_000_000_000)
    ns['publish_surroundings'](obj, single, struct.pack('<I i I', 256, 123, 456))
    assert len(messages) == 2


def test_acquisition_pose_interpolation_and_cloud_deskew():
    import math
    from geometry_msgs.msg import PoseStamped
    tree = ast.parse(pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('pose_at', 'publish_surroundings')]
    ns = dict(np=np, struct=struct, math=math, PoseStamped=PoseStamped, PointCloud2=PointCloud2, PointField=PointField)
    exec(compile(ast.Module(body=methods, type_ignores=[]), 'cloud', 'exec'), ns)
    clouds, poses = [], []
    # A robot translating 1 m/s and turning 1 rad/s, with bracketing odometry.
    obj = NS(pose_history=[(10., 0, 0, 0, 1, 0, 1), (10.1, .1, 0, .1, 1, 0, 1)],
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=11_000_000_000)),
             cloud_pub=NS(get_subscription_count=lambda: 1, publish=clouds.append),
             cloud_pose_pub=NS(publish=poses.append), cloud_last_ns=0,
             cloud_R=np.eye(3), cloud_t=np.zeros(3))
    obj.pose_at = lambda stamps: ns['pose_at'](obj, stamps)
    middle = obj.pose_at(np.array([10.05]))
    np.testing.assert_allclose([v[0] for v in middle], [.05, 0, .05])
    assert obj.pose_at(np.array([10.19])) is None
    assert obj.pose_at(np.array([9.9])) is None
    # Same world point (2,0,.3) measured at the frame's start and end.
    pts = np.zeros(2, dtype=[('x','f4'),('y','f4'),('z','f4'),('t','u4')])
    pts['x'] = [2, 1.9 * math.cos(.1)]
    pts['y'] = [0, -1.9 * math.sin(.1)]
    pts['z'] = .3; pts['t'] = [0, 100_000_000]
    ns['publish_surroundings'](obj, pts, struct.pack('<I i I', 256, 10, 0))
    xyz = np.frombuffer(clouds[0].data, dtype='<f4').reshape(-1,3)
    np.testing.assert_allclose(xyz, [[1.9*math.cos(.1), -1.9*math.sin(.1), .3]], atol=1e-6)
    assert clouds[0].header.stamp == poses[0].header.stamp
    assert poses[0].header.frame_id == 'odom'
    with tempfile.TemporaryDirectory() as directory:
        cloud_path, pose_path = pathlib.Path(directory)/'cloud.cdr', pathlib.Path(directory)/'pose.cdr'
        cloud_path.write_bytes(serialize_message(clouds[0])); pose_path.write_bytes(serialize_message(poses[0]))
        subprocess.run(['node', 'tests/web/surroundings_cdr_check.cjs', str(cloud_path), str(pose_path)], check=True)
