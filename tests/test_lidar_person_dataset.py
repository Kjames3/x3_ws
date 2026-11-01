"""Lidar person dataset tools: cluster features, PCD/LZF reading, auto-labelling rules."""
import pathlib
import struct
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src' / 'yahboomcar_bringup'))

from yahboomcar_bringup import cluster_features as cf  # noqa: E402
import lidar_person_dataset as lpd  # noqa: E402
import sit_pcd  # noqa: E402


def _person(cx, cy, n=400, seed=0):
    """Points on the near half of a 0.25 m radius, 1.75 m tall cylinder."""
    rng = np.random.default_rng(seed)
    a = np.arctan2(-cy, -cx) + rng.uniform(-1.2, 1.2, n)
    return cx + 0.25 * np.cos(a), cy + 0.25 * np.sin(a), rng.uniform(0.16, 1.75, n)


def test_features_person_shape():
    x, y, z = _person(3.0, 0.0)
    f = dict(zip(cf.NAMES, cf.features(x, y, z)))
    assert abs(f['range'] - 2.85) < 0.1
    assert 0.35 < f['width'] < 0.55 and f['depth'] < 0.25
    assert f['height'] > 1.4 and abs(sum(f[f'band_{i}'] for i in range(5)) - 1) < 1e-5
    assert f['bottom_gap'] > -0.01           # the Livox cannot see below ~0.0 at 2.35 m+


def test_features_scale_with_sensor_geometry():
    x, y, z = _person(2.0, 0.0)
    livox = dict(zip(cf.NAMES, cf.features(x, y, z)))
    high = dict(zip(cf.NAMES, cf.features(x, y, z, sensor={'sensor_z': 1.0, 'fov_top_tan': 0.268,
                                                            'fov_bottom_tan': 0.268})))
    assert livox['window_top'] == cf.Z_MAX_M     # +52 deg sees the whole person at 1.75 m
    assert high['window_top'] < 1.6 and high['window_bottom'] > 0.4


def test_crop_takes_the_candidate_box_only():
    x = np.array([1.0, 1.1, 1.5, 1.05])
    y = np.array([0.0, 0.1, 0.0, 0.05])
    z = np.array([1.0, 1.0, 1.0, 0.05])        # last one is below Z_MIN_M
    cand = (1.05, 0.05, 1.0, 1.1, 0.0, 0.1, 0.2, 1.8, 2)
    assert cf.crop(x, y, z, cand).tolist() == [True, True, False, False]


def test_lzf_literal_and_backref():
    # literal 'abc' (ctrl 2), then a back reference of length 4 at distance 3 -> 'abca'
    stream = bytes([2]) + b'abc' + bytes([(2 << 5) | 0, 2])
    assert sit_pcd.lzf_decompress(stream, 7) == b'abcabca'


def test_read_pcd_binary_compressed(tmp_path):
    xs = np.array([1.0, 2.0], dtype='<f4')
    ys = np.array([3.0, 4.0], dtype='<f4')
    payload = xs.tobytes() + ys.tobytes()                     # column-major
    lzf = bytes([len(payload) - 1]) + payload                 # one literal run
    head = (b'VERSION 0.7\nFIELDS x y\nSIZE 4 4\nTYPE F F\nCOUNT 1 1\nWIDTH 2\nHEIGHT 1\n'
            b'POINTS 2\nDATA binary_compressed\n')
    p = tmp_path / 'a.pcd'
    p.write_bytes(head + struct.pack('<II', len(lzf), len(payload)) + lzf)
    d = sit_pcd.read_pcd(p)
    assert d['x'].tolist() == [1.0, 2.0] and d['y'].tolist() == [3.0, 4.0]


def test_link_keeps_one_walker_on_one_tracklet():
    frames = [(0.1 * i, np.array([[1.0 + 0.1 * i, 0.0], [5.0, 5.0]])) for i in range(20)]
    ids = lpd.link(frames)
    assert len({int(f[0]) for f in ids}) == 1 and len({int(f[1]) for f in ids}) == 1


def test_labels_carry_along_the_track_and_need_a_clear_view_for_negatives():
    n = 20
    track = np.r_[np.zeros(n, int), np.ones(n, int), np.full(n, 2)]
    cam_hit = np.r_[np.r_[np.ones(5, bool), np.zeros(n - 5, bool)], np.zeros(2 * n, bool)]
    in_view = np.r_[np.r_[np.ones(5, bool), np.zeros(n - 5, bool)], np.ones(2 * n, bool)]
    cam_near = np.r_[np.zeros(2 * n, bool), np.ones(n, bool)]         # track 2: a person next to it
    xy = np.r_[np.c_[np.linspace(1, 6, n), np.zeros(n)],               # walker, out to 6 m
               np.tile([[2.0, 1.0]], (n, 1)), np.tile([[2.0, -1.0]], (n, 1))]
    label, source = lpd.label_tracklets(track, cam_hit, in_view, cam_near, xy)
    assert (label[:n] == 1).all() and (source[:5] == 'cam').all() and (source[5:n] == 'carried').all()
    assert (label[n:2 * n] == 0).all() and (source[n:2 * n] == 'static_neg').all()
    assert (label[2 * n:] == -1).all()


def test_camera_outliers_are_dropped():
    recs = []
    for i in range(12):
        x = 2.0 + 0.05 * i + (1.0 if i == 6 else 0.0)           # one raw-odom placement
        recs.append({'camera': [{'id': 7, 'state_stamp_s': 0.1 * i, 'obs_age_s': 0.0,
                                 'odom_x': x, 'odom_y': 0.0, 'measured': True}]})
    assert lpd.clean_camera(recs) == {('7', 0.6)}


def test_overrides_relabel_only_their_run_and_radius():
    xy = np.array([[0.0, 0.0], [0.3, 0.0], [2.0, 0.0]])
    label = np.array([0, 0, 0], dtype=np.int8)
    source = np.array(['persistent'] * 3, dtype=object)
    ov = [{'run': 'a', 'x': 0.0, 'y': 0.0, 'r_m': 0.45, 'label': 'sitting'},
          {'run': 'b', 'x': 2.0, 'y': 0.0, 'label': 'person'}]
    label, source = lpd.apply_overrides('a', xy, label, source, ov)
    assert label.tolist() == [2, 2, 0] and source[0] == 'override_sitting' and source[2] == 'persistent'
