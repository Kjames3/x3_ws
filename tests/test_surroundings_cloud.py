import ast, pathlib, struct, subprocess, tempfile
from array import array

import pytest
import numpy as np
from types import SimpleNamespace as NS


@pytest.fixture
def ros_cloud():
    messages = pytest.importorskip('sensor_msgs.msg')
    serialization = pytest.importorskip('rclpy.serialization')
    return messages.PointCloud2, messages.PointField, serialization.serialize_message


def _display_defs(tree):
    """Module-level display voxelizer (constants + helpers) for exec'd tests."""
    names={'NEAR_RADIUS','NEAR_CELL','FAR_CELL','DISPLAY_CAP','_first_per_cell','display_voxels','_cloud_data'}
    out=[]
    for n in tree.body:
        if isinstance(n,ast.FunctionDef) and n.name in names: out.append(n)
        elif isinstance(n,ast.Assign) and any(getattr(t,'id',None) in names or
                (isinstance(t,ast.Tuple) and any(getattr(e,'id',None) in names for e in t.elts)) for t in n.targets): out.append(n)
    return out


def _display_ns():
    tree=ast.parse(pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py').read_text())
    ns=dict(np=np, array=array); exec(compile(ast.Module(body=_display_defs(tree),type_ignores=[]),'dv','exec'),ns)
    return ns


def test_display_voxels_near_fine_far_coarse_and_capped():
    ns=_display_ns(); dv=ns['display_voxels']
    # Two points 3 cm apart: separate 2.5 cm cells near, one 5 cm cell far.
    near=np.array([[1.005,.005,.2],[1.035,.005,.2]]); far=near+[3,0,0]
    assert len(dv(near))==2 and len(dv(far))==1
    # Over the cap: total bounded, far shell keeps at least a third.
    rng=np.random.default_rng(0)
    big=np.vstack([rng.uniform(-2,2,(40000,3)), rng.uniform(3.2,5.5,(40000,3))])
    out=dv(big,cap=9000); r=np.hypot(out[:,0],out[:,1])
    assert len(out)==9000 and (r>=3).sum()>=3000
    assert ns['DISPLAY_CAP']==12000


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_fast_voxelization_matches_original_bytes(dtype):
    ns = _display_ns()
    rng = np.random.default_rng(42)
    # Distinct points in shared cells, including negative coordinates and
    # sparse and dense regions. Reverse the inputs to vary the winning rows.
    dense = rng.uniform(-.2, .2, (5_000, 3))
    sparse = rng.uniform(-6, 6, (15_000, 3))
    xyz = np.concatenate((dense, sparse)).astype(dtype)
    rng.shuffle(xyz)
    for points in (xyz, xyz[::-1], xyz[:0], xyz[:1], np.repeat(xyz[:1], 8, axis=0)):
        for cell in (.025, .05, .2):
            old = ns['_first_per_cell'](points, cell)
            fast = ns['_first_per_cell'](points, cell, fast=True)
            assert fast.shape == old.shape and fast.dtype == old.dtype
            assert fast.tobytes() == old.tobytes()
        for cap in (1000, 12000, 20000):
            old = ns['display_voxels'](points, cap=cap)
            fast = ns['display_voxels'](points, cap=cap, fast=True)
            assert fast.shape == old.shape and fast.dtype == old.dtype
            assert fast.tobytes() == old.tobytes()


def test_fast_payload_matches_pointcloud2_serialization(ros_cloud):
    PointCloud2, _, serialize_message = ros_cloud
    ns = _display_ns()
    xyz = np.random.default_rng(42).normal(size=(50, 3)).astype('<f4')
    for rows in (xyz, xyz[:0], xyz[::-2], xyz.astype('>f4')):
        old = PointCloud2()
        fast = PointCloud2()
        old.data = ns['_cloud_data'](rows)
        payload = ns['_cloud_data'](rows, fast=True)
        fast.data = payload
        assert fast.data is payload and fast.data.typecode == 'B'
        assert serialize_message(fast) == serialize_message(old)


@pytest.mark.parametrize('fast', [False, True])
def test_publisher_browser_roundtrip(fast, ros_cloud):
    PointCloud2, PointField, serialize_message = ros_cloud
    path=pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py')
    tree=ast.parse(path.read_text()); cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish_surroundings')
    ns=dict(np=np,struct=struct,PointCloud2=PointCloud2,PointField=PointField,
            array=array, FAST_CLOUD=fast)
    exec(compile(ast.Module(body=_display_defs(tree)+[method],type_ignores=[]),'cloud','exec'),ns)
    messages=[]
    clock=NS(now=lambda:NS(nanoseconds=1_000_000_000))
    obj=NS(get_clock=lambda:clock, cloud_pub=NS(get_subscription_count=lambda:1,publish=messages.append),pose_at=lambda stamps: None,cloud_last_ns=0,cloud_R=np.eye(3),cloud_t=np.array([.0432,0,.31]),display_cap=ns['DISPLAY_CAP'])
    rng=np.random.default_rng(1); pts=np.zeros(20000,dtype=[('x','f4'),('y','f4'),('z','f4'),('t','u4')]); xyz=rng.uniform(-4,4,(20000,3))
    for i,k in enumerate('xyz'):pts[k]=xyz[:,i]
    pts['x'][0]=float('nan');pts['x'][1]=pts['y'][1]=pts['z'][1]=0
    ns['publish_surroundings'](obj,pts,struct.pack('<I i I',256,123,456))
    m=messages[0];assert 0<m.width<=12000 and m.header.frame_id=='base_footprint'
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


def test_acquisition_pose_interpolation_and_cloud_deskew(ros_cloud):
    PointCloud2, PointField, serialize_message = ros_cloud
    import math
    from geometry_msgs.msg import PoseStamped
    tree = ast.parse(pathlib.Path('src/yahboomcar_bringup/yahboomcar_bringup/livox_scan_node.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('pose_at', 'publish_surroundings')]
    ns = dict(np=np, struct=struct, math=math, PoseStamped=PoseStamped, PointCloud2=PointCloud2,
              PointField=PointField, array=array, FAST_CLOUD=False)
    exec(compile(ast.Module(body=_display_defs(tree) + methods, type_ignores=[]), 'cloud', 'exec'), ns)
    clouds, poses = [], []
    # A robot translating 1 m/s and turning 1 rad/s, with bracketing odometry.
    obj = NS(pose_history=[(10., 0, 0, 0, 1, 0, 1), (10.1, .1, 0, .1, 1, 0, 1)],
             get_clock=lambda: NS(now=lambda: NS(nanoseconds=11_000_000_000)),
             cloud_pub=NS(get_subscription_count=lambda: 1, publish=clouds.append),
             cloud_pose_pub=NS(publish=poses.append), cloud_last_ns=0,
             cloud_R=np.eye(3), cloud_t=np.zeros(3), display_cap=ns['DISPLAY_CAP'])
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
