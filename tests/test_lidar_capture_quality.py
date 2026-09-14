import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lidar_capture_quality import sweep_quality


def fixture():
    t = np.arange(0, 10, .14)
    jt = np.arange(-.1, 10.2, .05)
    pitch = np.deg2rad(45 * np.sin(jt * np.pi / 2))
    return np.column_stack([np.arange(len(t)), np.zeros(len(t)), t, t*0+100]), np.column_stack([jt, pitch, jt*0])


def test_full_sweep_passes():
    assert sweep_quality(*fixture())[0]


def test_partial_step_sweep_rejected_despite_many_clouds():
    clouds, joints = fixture()
    joints[:, 1] = np.deg2rad(np.linspace(-44, -32, len(joints)))
    assert not sweep_quality(clouds, joints)[0]


def test_sparse_clouds_and_missing_joint_brackets_rejected():
    clouds, joints = fixture()
    assert not sweep_quality(np.delete(clouds, np.arange(15, 25), axis=0), joints)[0]
    assert not sweep_quality(clouds, joints[10:])[0]
    assert not sweep_quality(clouds, np.delete(joints, np.arange(50, 65), axis=0))[0]


def test_nonmonotonic_time_rejected():
    clouds, joints = fixture()
    clouds[20, 2] = clouds[19, 2]
    assert not sweep_quality(clouds, joints)[0]
