"""One person, one box: the merge rule and the live driver's use of it."""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from person_box_merge import contain, group_overlapping, union_box  # noqa: E402


def test_split_person_from_the_lab_becomes_one_group():
    # approach-r1, 2026-09-29: left half, right half and whole body.
    boxes = [[0, 2, 187, 513], [174, 0, 380, 513], [0, 0, 371, 521]]
    assert group_overlapping(boxes) == [[0, 1, 2]]
    assert union_box(boxes) == [0, 0, 380, 521]


def test_near_duplicate_boxes_merge_and_separate_people_do_not():
    assert group_overlapping([[125, 0, 378, 496], [122, 0, 379, 490]]) == [[0, 1]]
    assert group_overlapping([[0, 0, 100, 300], [200, 0, 300, 300]]) == [[0], [1]]
    # Side by side with a small overlap: still two people.
    assert contain([0, 0, 100, 300], [80, 0, 180, 300]) < 0.6
    assert group_overlapping([[0, 0, 100, 300], [80, 0, 180, 300]]) == [[0], [1]]


def _cand(label, conf, box, mask=None):
    return {'label': label, 'conf': conf, 'box': list(box), 'mask': mask, 'kpts': None}


def test_driver_merges_person_candidates_only_and_ors_the_masks():
    from oakd_driver import OakDCamera
    left = np.zeros((6, 8), bool); left[:, :4] = True
    right = np.zeros((6, 8), bool); right[:, 3:] = True
    cands = [_cand('person', 0.6, [0, 0, 4, 6], left),
             _cand('chair', 0.9, [0, 0, 8, 6]),          # overlaps everything, not a person
             _cand('person', 0.8, [0, 0, 8, 6], None),   # whole body, mask unreadable
             _cand('person', 0.7, [3, 0, 8, 6], right),
             _cand('person', 0.5, [20, 0, 28, 6], left)]  # someone else
    out = OakDCamera._merge_people(cands)
    assert [c['label'] for c in out] == ['person', 'chair', 'person']
    merged = out[0]
    assert merged['box'] == [0, 0, 8, 6] and merged['conf'] == 0.8 and merged['merged_from'] == 3
    assert merged['mask'].all()
    assert out[2]['box'] == [20, 0, 28, 6] and 'merged_from' not in out[2]


def test_driver_leaves_unsplit_detections_untouched():
    from oakd_driver import OakDCamera
    cands = [_cand('person', 0.9, [0, 0, 10, 30]), _cand('person', 0.8, [50, 0, 60, 30])]
    assert OakDCamera._merge_people(cands) is cands
