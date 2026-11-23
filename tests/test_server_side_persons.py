"""server_x3 X3_SIDE_PERSONS wiring: side rows -> tracker candidates + camera anchors.

The ROS2Bridge methods are pulled out of server_x3.py by AST and run on a stub, so this
needs neither ROS nor the hardware."""
import ast
import math
import pathlib
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src' / 'yahboomcar_bringup')]

import lidar_person_tracker as lpt  # noqa: E402
from yahboomcar_bringup import person_candidates as pc  # noqa: E402

SIDE_FIELDS = pc.FIELDS + ('ax', 'ay', 'cam', 'tid', 'age')
METHODS = ('_person_candidates_cb', '_side_candidates_cb', 'publish_side_rays', 'static_chain',
           'pose_at_monotonic', '_lidar_persons_record')


def _bridge_ns(side_on=True):
    src = (ROOT / 'src' / 'server_x3.py').read_text()
    cls = next(n for n in ast.parse(src).body if isinstance(n, ast.ClassDef) and n.name == 'ROS2Bridge')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    assert {m.name for m in methods} == set(METHODS)
    logs = []
    ns = dict(np=np, math=math, time=time, json=__import__('json'), merge_seeded=lpt.merge_seeded,
              c3_live=None, SEEDED_TRACKER=False, SIDE_PERSONS=side_on, SIDE_ROWS_MAX_AGE_S=0.05,
              CAMERA_ROLES=('front', 'rear', 'left', 'right'),
              _lidar_persons=lpt.LidarPersonTracker(), _lidar_persons_lock=threading.Lock(),
              _lidar_persons_logged={'t': 0.0, 'stats': None, 'rec_t': 1e18, 'rec_path': None, 'rec': None},
              LIDAR_PERSONS_RECORD_FLAG='/nonexistent', logger=NS(info=lambda *a: logs.append(a),
                                                                 warning=lambda *a: logs.append(a)))
    exec(compile(ast.Module(body=methods, type_ignores=[]), 'server_x3', 'exec'), ns)
    return ns, logs


def _msg(fields, rows, stamp):
    rows = np.asarray(rows, dtype='<f4').reshape(-1, len(fields))
    sec = int(stamp)
    return NS(fields=[NS(name=f) for f in fields], data=rows.tobytes(), width=len(rows),
              header=NS(stamp=NS(sec=sec, nanosec=int(round((stamp - sec) * 1e9)))))


def _bridge(ns):
    b = NS(_seeded={'rows': None}, _side={'stamp': None, 'rows': None, 'fields': None},
           get_pose_m=lambda: {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    for name in METHODS:
        fn = ns[name]
        setattr(b, name, (lambda f: (lambda *a, **k: f(b, *a, **k)))(fn) if name != '_lidar_persons_record'
                else fn)
    return b


def _cand_row(x, y):
    return [x, y, x - .2, x + .2, y - .2, y + .2, 0.15, 1.7, 300]


def test_side_rows_birth_a_track_with_a_namespaced_id():
    ns, logs = _bridge_ns()
    b = _bridge(ns)
    t = 1000.0
    for i in range(5):
        now = t + 0.1 * i
        # the side body duplicates the ordinary candidate: anchor only, no extra candidate
        b._side_candidates_cb(_msg(SIDE_FIELDS, [_cand_row(0.02, 2.5) + [0.0, 2.5, 2, 7, 0.05]], now))
        b._person_candidates_cb(_msg(pc.FIELDS, [_cand_row(0.0, 2.5)], now))
    res = ns['_lidar_persons'].results
    assert not [l for l in logs if 'failed' in str(l[0])], logs
    assert len(res) == 1 and res[0]['confirmed'] and res[0]['camera_id'] == 'left:7'
    assert ns['_lidar_persons'].stats['side_births'] == 1


def test_side_body_missing_from_ordinary_candidates_is_added():
    """A seated person extract rejects (merged with the desk) exists only as a side row."""
    ns, _ = _bridge_ns()
    b = _bridge(ns)
    for i in range(5):
        now = 50.0 + 0.1 * i
        b._side_candidates_cb(_msg(SIDE_FIELDS, [_cand_row(0.0, -2.0) + [0.0, -2.0, 3, 1, 0.1]], now))
        b._person_candidates_cb(_msg(pc.FIELDS, [_cand_row(3.0, 0.0)], now))
    res = ns['_lidar_persons'].results
    assert len(res) == 1 and res[0]['camera_id'] == 'right:1'
    assert res[0]['odom_y'] == pytest.approx(-2.0, abs=0.05)


def test_flag_off_or_stale_rows_change_nothing():
    for side_on, stamp_off in ((False, 0.0), (True, 0.2)):
        ns, _ = _bridge_ns(side_on)
        b = _bridge(ns)
        for i in range(5):
            now = 50.0 + 0.1 * i
            b._side_candidates_cb(_msg(SIDE_FIELDS, [_cand_row(0.0, -2.0) + [0.0, -2.0, 3, 1, 0.1]],
                                       now - stamp_off))
            b._person_candidates_cb(_msg(pc.FIELDS, [_cand_row(0.0, -2.0)], now))
        assert ns['_lidar_persons'].results == [] and ns['_lidar_persons'].stats['births'] == 0


def test_publish_side_rays_message_layout():
    import side_person_rays as spr
    ns, _ = _bridge_ns()
    sent = []
    b = _bridge(ns)
    b._side_ray_pubs = {'left': NS(publish=sent.append)}
    b._PointCloud2 = lambda: NS(header=NS(stamp=NS(sec=0, nanosec=0), frame_id=''))
    b._PointField = lambda **k: NS(**k)
    b._PointField.FLOAT32 = 7
    row = [1.0, 2.0, 0.21, 0.0, 1.0, 0.4, 6.0, float('nan'), 3.0]
    b.publish_side_rays('left', [row], 123.25)
    m = sent[0]
    assert [f.name for f in m.fields] == list(spr.RAY_MSG_FIELDS) and m.width == 1
    assert m.header.frame_id == 'odom' and (m.header.stamp.sec, m.header.stamp.nanosec) == (123, 250000000)
    got = np.frombuffer(m.data, dtype='<f4')
    assert np.allclose(got[:7], row[:7]) and math.isnan(got[7]) and got[8] == 3.0
    b.publish_side_rays('left', [], 124.0)               # an empty message clears the camera
    assert sent[1].width == 0 and sent[1].data == b''


def test_static_chain_composes_to_base_footprint():
    ns, _ = _bridge_ns()
    b = _bridge(ns)
    Rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    b._static_chains = {}
    b._static_tf = {'cam': ('livox_frame', Rz, np.array([0.1, 0, 0])),
                    'livox_frame': ('base_link', np.eye(3), np.array([0, 0, 0.2])),
                    'base_link': ('base_footprint', np.eye(3), np.array([0, 0, 0.08]))}
    R, t = b.static_chain('cam')
    assert np.allclose(R, Rz) and np.allclose(t, [0.1, 0, 0.28])
    assert b.static_chain('nowhere') is None
