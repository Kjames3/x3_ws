"""yolo26n plain head: lock the settled anchor-major layout, don't guess.

Real tensors (2026-08 on depthai 2.32, 2026-11-16 on 3.11) are [6300, 85]
anchor-major: ch0..3 box in input pixels, ch4 == max(ch5:85) byte-exact,
ch5:85 the 80 COCO classes. On an empty scene the hit-count guesser rejected
that layout (0 hits) and locked a channel-major reading, turning box pixels
into 'cake'/'person' at confidence 1.0.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

import oakd_driver  # noqa: E402

CFG = ROOT / 'src' / 'blobs' / 'yolo26n' / 'config.json'


class _NN:
    def __init__(self, flat):
        self.flat = flat

    def getLayerFp16(self, name):
        return self.flat

    def getFirstLayerFp16(self):
        return self.flat


def _tensor(seed=0, objects=()):
    """[6300, 85] like the real export: boxes in pixels, class scores <= 0.1."""
    rng = np.random.default_rng(seed)
    A = np.zeros((6300, 85), np.float32)
    A[:, 0] = rng.uniform(0, 480, 6300)
    A[:, 1] = rng.uniform(0, 640, 6300)
    A[:, 2] = rng.uniform(2, 480, 6300)
    A[:, 3] = rng.uniform(2, 640, 6300)
    A[:, 5:85] = rng.uniform(0, 0.1, (6300, 80))
    for anchor, cls, score, box in objects:
        A[anchor, 0:4] = box
        A[anchor, 5 + cls] = score
    A = A.astype(np.float16).astype(np.float32)
    A[:, 4] = A[:, 5:85].max(axis=1)
    return A.ravel()


def _cam():
    cam = oakd_driver.OakDCamera(spatial_config=str(CFG))
    assert cam._nn_rows == 85 and not cam.nn_kpts and not cam.nn_masks
    return cam


def test_empty_scene_locks_the_dup_layout_and_reports_nothing():
    cam = _cam()
    cam._process_nn(_NN(_tensor()))
    assert cam._decode_mode == ('am', 'dup')
    assert cam.get_spatial_detections() == []


def test_labels_come_from_classes_5_to_85_without_squaring():
    cam = _cam()
    dog = 16
    cam._process_nn(_NN(_tensor(objects=[(100, dog, 0.8, (240, 320, 120, 160))])))
    dets = cam.get_spatial_detections()
    assert cam._decode_mode == ('am', 'dup')
    assert len(dets) == 1
    assert dets[0]['label'] == cam.labels[dog]
    assert dets[0]['label'] != 'person'          # the old ch4-tie bug labelled everything person
    assert abs(dets[0]['conf'] - 0.8) < 2e-3     # not 0.64 (obj * cls squaring)


def test_other_layouts_still_fall_back_to_the_guesser():
    cam = _cam()
    raw = _tensor().reshape(6300, 85)
    raw[:, 4] = 0.0                              # invariant broken
    assert not cam._dup_layout(raw.ravel())


def test_anchor_count_from_the_anchor_major_config():
    assert _cam()._N_ANCHORS == 6300             # config shape [1, 6300, 85], not 85
