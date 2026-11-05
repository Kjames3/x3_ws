import json
import math

import numpy as np

import tof_exempt_zones as z

SQUARE = np.array([[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]])


def test_point_in_polygon():
    m = z.points_in_polygon([[1.5, 1.5], [0.5, 1.5], [2.5, 2.5]], SQUARE)
    assert m.tolist() == [True, False, False]


def test_map_transform_round_trip():
    # Robot at odom (1, 0, 90deg); AMCL says it is at map (5, 5, 0).
    mfo = z.compose_map_from_odom((5.0, 5.0, 0.0), (1.0, 0.0, math.pi / 2))
    # A point 0.3 m ahead of the robot sits at map (5.3, 5).
    m = z.base_to_map([[0.3, 0.0]], (1.0, 0.0, math.pi / 2), mfo)
    assert np.allclose(m, [[5.3, 5.0]])


def test_exempt_mask_only_inside_zone():
    mfo = (0.0, 0.0, 0.0)
    pose = (1.2, 1.5, 0.0)                 # robot at map (1.2, 1.5) facing +x
    mask = z.exempt_mask([[0.3, 0.0], [1.2, 0.0]], pose, mfo, [SQUARE])
    assert mask.tolist() == [True, False]
    assert not z.exempt_mask([[0.3, 0.0]], pose, None, [SQUARE]).any()


def test_load_zones(tmp_path):
    p = tmp_path / 'z.json'
    p.write_text(json.dumps({'_comment': 'x', 'RealLab3.yaml': [SQUARE.tolist()],
                             'Bad': [[[0, 0], [1, 1]]], 'Empty': []}))
    zones = z.load_zones(str(p))
    assert list(zones) == ['RealLab3']
    assert z.load_zones(str(tmp_path / 'missing.json')) == {}
    assert z.map_stem('/a/b/RealLab3.yaml') == 'RealLab3'
