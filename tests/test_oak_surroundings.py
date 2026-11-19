import numpy as np
from types import SimpleNamespace as NS
from oak_surroundings import transform_xyz


def test_transform_optical_points_uses_rotation_then_translation():
    t=NS(rotation=NS(x=0,y=0,z=np.sqrt(.5),w=np.sqrt(.5)),translation=NS(x=1,y=2,z=3))
    np.testing.assert_allclose(transform_xyz(np.array([[1.,0.,0.],[0.,1.,0.]]),t),[[1,3,3],[0,2,3]],atol=1e-6)


def test_old_camera_pair_is_rejected_before_transform_or_publish():
    import pytest,time
    pytest.importorskip('rclpy')
    from oak_surroundings import OakSurroundings
    obj=OakSurroundings.__new__(OakSurroundings)
    pair={'rgb':{'meta':{'session_id':'test'}},'depth':{'meta':{'host_monotonic_estimate_ns':time.monotonic_ns()-2_000_000_000}},'epoch':0,'pair_seq':1,'calibration':{'depth_recorded':'raw_uncorrected'}}
    obj.camera=NS(get_display_rgbd_pair=lambda:pair);obj.last=None
    # No node/TF/publisher attached: touching any would fail this test.
    assert obj.publish() is None


# Exercise the actual publisher with real ROS messages but no device or ROS node.
import pytest


@pytest.fixture
def publisher(monkeypatch):
    pytest.importorskip('rclpy')
    from sensor_msgs.msg import PointCloud2, PointField
    from geometry_msgs.msg import PoseStamped, TransformStamped
    import oak_surroundings as module
    clock = {'ns': 10_000_000_000}
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: clock['ns'])
    obj = module.OakSurroundings.__new__(module.OakSurroundings)
    k = [100., 0., 20., 0., 100., 20., 0., 0., 1.]
    pair = {'rgb': {'meta': {'session_id': 'one'}, 'image': np.full((40,40,3), [10,20,30], np.uint8)},
            'depth': {'meta': {'host_monotonic_estimate_ns': 9_900_000_000}, 'image': np.full((40,40),1000,np.uint16)},
            'epoch': 0, 'pair_seq': 1, 'calibration': {'depth_recorded': 'raw_uncorrected', 'width':40, 'height':40,
            'rgb_k':k, 'depth_k':k, 'rgb_d':[0.]*5, 'depth_inv_offset_per_m':.0821}}
    obj.camera = NS(get_display_rgbd_pair=lambda:pair)
    obj.node = NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock['ns'])))
    capture = TransformStamped(); capture.transform.rotation.w = 1.; capture.transform.translation.x = 2.
    pose = TransformStamped(); pose.header.stamp.sec = 10; pose.transform.rotation.w = 1.
    calls = []
    def lookup(target, source, stamp):
        calls.append((target, source, stamp.nanoseconds))
        return capture if source == 'oak_rgb_camera_optical_frame' else pose
    obj.tf = NS(lookup_transform=lookup)
    clouds, poses = [], []
    obj.pub = NS(publish=clouds.append); obj.pose_pub = NS(publish=poses.append)
    obj.Cloud, obj.Field, obj.Pose = PointCloud2, PointField, PoseStamped
    obj.last = obj.maps_key = obj.maps = obj.offset = None
    obj.stage_samples = []
    return obj, pair, clock, clouds, poses, calls, pose


def test_publish_capture_stamp_colors_duplicate_and_bounded_timings(publisher):
    obj, pair, clock, clouds, poses, calls, pose = publisher
    assert obj.publish() == 100.
    msg = clouds[0]
    assert calls[0][2] == 9_900_000_000
    assert msg.header.frame_id == 'odom' and msg.header.stamp.nanosec == 900_000_000
    data = np.frombuffer(msg.data, dtype=[('xyz','<f4',(3,)),('rgb','<u4')])
    assert 0 < len(data) <= 6000
    assert np.isfinite(data['xyz']).all()
    np.testing.assert_allclose(data['xyz'][:,2], 1/1.0821, rtol=1e-6)
    assert np.all(data['xyz'][:,0] > 1.)  # capture transform applied
    assert 30<<16 | 20<<8 | 10 in data['rgb']
    assert len(obj.stage_samples[0]) == 7 and obj.stage_samples[0][-1] == 100.
    assert obj.publish() is None and len(clouds) == 1
    for seq in range(2,135):
        pair['pair_seq'] = seq; obj.publish()
    assert len(obj.stage_samples) == 128
    pair['rgb']['meta']['session_id'] = 'reconnected'
    obj.publish()
    assert len(clouds) == 135


def test_stale_pose_and_clock_jump_do_not_publish(publisher):
    obj, pair, clock, clouds, poses, calls, pose = publisher
    pose.header.stamp.sec = 8
    assert obj.publish() is None and not clouds
    pose.header.stamp.sec = 10
    obj.offset = -200_000_000
    assert obj.publish() is None and not clouds


def test_processing_that_becomes_stale_drops_cloud(publisher, monkeypatch):
    import oak_surroundings as module
    obj, pair, clock, clouds, poses, calls, pose = publisher
    original = module.refine
    def slow(*args, **kwargs):
        result = original(*args, **kwargs)
        clock['ns'] += 600_000_000
        return result
    monkeypatch.setattr(module, 'refine', slow)
    assert obj.publish() is None
    assert not clouds and not poses and not obj.stage_samples


def test_missing_capture_transform_cannot_publish(publisher):
    obj, pair, clock, clouds, poses, calls, pose = publisher
    def missing(*args):
        raise LookupError('capture pose unavailable')
    obj.tf.lookup_transform = missing
    # run() catches lookup failures; publish() must emit neither message.
    with pytest.raises(LookupError):
        obj.publish()
    assert not clouds and not poses


@pytest.mark.parametrize('change', ['future', 'corrected', 'dimensions'])
def test_invalid_input_does_not_publish(publisher, change):
    obj, pair, clock, clouds, poses, calls, pose = publisher
    if change == 'future':
        pair['depth']['meta']['host_monotonic_estimate_ns'] = clock['ns'] + 1
    elif change == 'corrected':
        pair['calibration']['depth_recorded'] = 'corrected'
    else:
        pair['rgb']['image'] = pair['rgb']['image'][:20]
    assert obj.publish() is None
    assert not clouds and not poses


def test_process_route_preserves_capture_timestamp_and_geometry(publisher):
    import oak_surroundings as module
    obj,pair,clock,clouds,poses,calls,pose=publisher
    obj.publish();baseline=bytes(clouds[-1].data)
    pair['pair_seq']=2
    seen=[]
    def compute(inputs,source,timings):
        seen.append(source)
        return module.refine(*inputs)
    obj.process_enabled=True;obj.geometry_worker=NS(compute=compute)
    obj.publish()
    assert seen==[9_900_000_000]
    assert bytes(clouds[-1].data)==baseline
    assert clouds[-1].header.stamp==clouds[0].header.stamp


def test_process_failure_closes_and_suppresses_publish(publisher):
    obj,pair,clock,clouds,poses,calls,pose=publisher
    closed=[]
    def fail(*args): raise TimeoutError('test')
    obj.process_enabled=True;obj.geometry_worker=NS(compute=fail,close=lambda:closed.append(True))
    with pytest.raises(TimeoutError): obj.publish()
    assert closed and obj.geometry_worker is None and obj.worker_retry_at>0
    assert not clouds and not poses
    assert obj.publish() is None
