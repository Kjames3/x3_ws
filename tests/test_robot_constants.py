"""Guard the URDF and config files against accidental edits.

Two kinds of value live here.  Settled physical facts (CLAUDE.md, "do not
re-derive") are pinned exactly.  Values that legitimately change when the
robot is re-calibrated are only range- and shape-checked, so re-recording a
baseline or re-levelling the mount never needs a test edit.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
URDF = REPO / 'src/yahboomcar_description/urdf/yahboomcar_X3.urdf'
CONFIG = REPO / 'config'


# ---------------------------------------------------------------------- URDF
@pytest.fixture(scope='module')
def urdf():
    return ET.parse(URDF).getroot()


def _joint(urdf, name):
    j = urdf.find(f"joint[@name='{name}']")
    assert j is not None, f'joint {name} missing from the URDF'
    return j


def _origin(joint):
    o = joint.find('origin')
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return xyz, rpy


def test_urdf_is_a_single_tree(urdf):
    links = [l.get('name') for l in urdf.findall('link')]
    assert len(links) == len(set(links)), 'duplicate link names'
    joints = urdf.findall('joint')
    names = [j.get('name') for j in joints]
    assert len(names) == len(set(names)), 'duplicate joint names'
    children = []
    for j in joints:
        parent, child = j.find('parent').get('link'), j.find('child').get('link')
        assert parent in links, f"{j.get('name')}: parent {parent} is not a link"
        assert child in links, f"{j.get('name')}: child {child} is not a link"
        children.append(child)
    assert len(children) == len(set(children)), 'a link has two parent joints'
    roots = set(links) - set(children)
    assert roots == {'base_footprint'}, f'expected one root, got {sorted(roots)}'


def test_urdf_meshes_exist(urdf):
    pkg = REPO / 'src'
    for mesh in urdf.iter('mesh'):
        name = mesh.get('filename')
        if name.startswith('package://'):
            assert (pkg / name[len('package://'):]).is_file(), f'missing mesh {name}'


def test_base_joint_height(urdf):
    # Caliper-measured 2026-09-13: plate 27.5 mm off the floor.
    xyz, rpy = _origin(_joint(urdf, 'base_joint'))
    assert xyz == [0.0, 0.0, 0.0815]
    assert rpy == [0.0, 0.0, 0.0]


def test_laser_joint_keeps_its_half_turn(urdf):
    # Raw laser +X points at the robot's rear.  Forgetting this is what
    # produced the wrong tilt_direction on 2026-09-04.
    _, rpy = _origin(_joint(urdf, 'laser_joint'))
    assert rpy[:2] == [0.0, 0.0]
    assert rpy[2] == pytest.approx(math.pi, abs=1e-6)


def test_lidar_tilt_axis_is_pitch(urdf):
    j = _joint(urdf, 'lidar_tilt_joint')
    assert j.get('type') == 'revolute'
    assert [float(v) for v in j.find('axis').get('xyz').split()] == [0.0, 1.0, 0.0]


@pytest.mark.parametrize('joint, down_deg', [('tof_upper_joint', 15.0), ('tof_lower_joint', 40.0)])
def test_tof_array_pitch(urdf, joint, down_deg):
    _, rpy = _origin(_joint(urdf, joint))
    assert 90.0 + math.degrees(rpy[1]) == pytest.approx(down_deg, abs=0.01)


# ------------------------------------------------------------------- configs
def test_every_config_json_parses():
    files = sorted(CONFIG.glob('*.json'))
    assert files
    for f in files:
        json.loads(f.read_text())


def test_dynamixel_tilt_calibration():
    cal = json.loads((CONFIG / 'lidar_tilt_calibration_dynamixel.json').read_text())
    # Settled: +1 means increasing counts = nose down (sign_comparison.npz).
    assert cal['tilt_direction'] == 1
    assert cal['direction_verification']['result'] == 1
    # X-series: 4096 counts per turn.
    assert cal['counts_per_deg'] == pytest.approx(4096 / 360, abs=1e-3)
    assert cal['port'] == '/dev/openrb150'
    assert cal['servo_id'] == 1
    # Re-calibratable.  The level point sits near mid-travel; anything else
    # means a remount or a Homing Offset, both of which need a human look.
    assert isinstance(cal['horizontal_counts'], int)
    assert 1800 <= cal['horizontal_counts'] <= 2300
    # Backlash leaves ~10 counts of error, so a tighter band warns every boot.
    assert 10 <= cal['tolerance_counts'] <= 30


def test_camera_ground_plane():
    cfg = json.loads((CONFIG / 'camera_ground_plane.json').read_text())
    assert 0.15 <= cfg['camera_height_m'] <= 0.30
    assert abs(cfg['camera_pitch_deg']) <= 5.0
    assert 0.0 < cfg['min_obstacle_height_m'] < cfg['max_obstacle_height_m']


def test_oak_depth_correction():
    cfg = json.loads((CONFIG / 'oak_depth_correction.json').read_text())
    assert cfg['devices']
    for mxid, dev in cfg['devices'].items():
        assert abs(dev['inv_depth_offset_per_m']) < 0.5, mxid


@pytest.mark.parametrize('path', sorted(CONFIG.glob('tof_floor_baseline*.json')),
                         ids=lambda p: p.name)
def test_tof_floor_baseline(path):
    rec = json.loads(path.read_text())
    assert rec['sensor'] == 'lower'
    assert rec['frames'] >= 100
    depth = rec['depth_mm']
    assert len(depth) == 64 and len(rec['valid_frac']) == 64
    # null marks a zone with no return (wood loses the far row); the server
    # turns it into NaN.  More than a row or two missing is a bad recording.
    seen = [d for d in depth if d is not None]
    assert len(seen) >= 48
    # The lower array looks at floor a few tens of cm ahead of the bumper.
    assert all(50.0 <= d <= 1500.0 for d in seen), (min(seen), max(seen))
    assert all(0.0 <= v <= 1.0 for v in rec['valid_frac'])


def test_default_tof_baseline_exists():
    # server_x3.py degrades silently to the height test if this file is gone.
    assert (CONFIG / 'tof_floor_baseline.json').is_file()
