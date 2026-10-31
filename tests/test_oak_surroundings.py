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
