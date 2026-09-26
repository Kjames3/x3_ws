"""Person-tag hysteresis on ObstacleTracker (phantom-pedestrian fix)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from velocity_estimator import (  # noqa: E402
    ObstacleTracker, PERSON_CONFIRM, PERSON_RELEASE_FRAMES)

C = [(0.0, 0.0, 2.0)]
G = [(2.0, 0.0, 2.0)]


def _run(flags_seq):
    tr = ObstacleTracker()
    out = None
    for flags in flags_seq:
        out = tr.update(C, G, flags)
    return next(iter(out.values()))["is_person"]


def test_no_detector_is_unknown():
    assert _run([None] * 5) is None


def test_static_blob_never_promoted():
    assert _run([[False]] * 10) is False


def test_person_confirms_after_n_frames():
    assert _run([[True]] * (PERSON_CONFIRM - 1)) is False
    assert _run([[True]] * PERSON_CONFIRM) is True


def test_brief_miss_keeps_person():
    assert _run([[True]] * 6 + [[False]] * 2) is True


def test_single_false_box_does_not_promote():
    assert _run([[False]] * 3 + [[True]] + [[False]] * 3) is False


def test_confirmed_person_survives_detector_dropout():
    # Walk-run pattern: YOLO misses a confirmed walker for several frames.
    assert _run([[True]] * 4 + [[False]] * (PERSON_RELEASE_FRAMES - 1)) is True


def test_confirmed_person_released_after_sustained_misses():
    assert _run([[True]] * 4 + [[False]] * PERSON_RELEASE_FRAMES) is False


def test_hit_resets_release_countdown():
    seq = ([[True]] * 4 + [[False]] * (PERSON_RELEASE_FRAMES - 1) + [[True]]
           + [[False]] * (PERSON_RELEASE_FRAMES - 1))
    assert _run(seq) is True


# --- Step 2: motion evidence / dynamic category -----------------------------
import math  # noqa: E402
import random  # noqa: E402

from velocity_estimator import (  # noqa: E402
    MOTION_CONFIRM, MOTION_RELEASE_FRAMES, WINDOW_SIZE, motion_evidence)


def _hist(points):
    return [(x, y, 2.0) for x, y in points]


def test_jitter_is_not_motion():
    rng = random.Random(0)
    # 0.3-1.3 m/s phantoms come from ~5-10 cm centroid wander; no net travel.
    for _ in range(200):
        h = _hist([(2.0 + rng.gauss(0, 0.06), rng.gauss(0, 0.06))
                   for _ in range(WINDOW_SIZE)])
        assert not motion_evidence(h, 2.0)


def test_roomba_speed_is_motion():
    # 0.3 m/s straight line, 10 Hz, with 1 cm noise.
    rng = random.Random(1)
    h = _hist([(2.0 + 0.03 * i + rng.gauss(0, 0.01), rng.gauss(0, 0.01))
               for i in range(WINDOW_SIZE)])
    assert motion_evidence(h, 2.0)


def test_far_blob_needs_more_travel():
    # Endpoint-mean separation is 7 steps = 0.245 m: over the 0.20 m floor, so
    # it passes at 1 m, but under 3*sigma_diff = 0.35 m at 4 m (sigma ~ Z^2).
    h = _hist([(0.035 * i, 0.0) for i in range(WINDOW_SIZE)])
    assert motion_evidence(h, 1.0)
    assert not motion_evidence(h, 4.0)


def test_short_history_is_not_motion():
    assert not motion_evidence(_hist([(0.1 * i, 0) for i in range(3)]), 1.0)


def _track_moving(n_move, n_still_after=0):
    tr = ObstacleTracker()
    out = None
    x = 1.5
    for i in range(WINDOW_SIZE + n_move + n_still_after):
        if i < WINDOW_SIZE + n_move:
            x = 1.5 + 0.04 * i
        out = tr.update([(0.0, 0.0, x)], [(x, 0.0, x)], [False])
    return next(iter(out.values()))


def test_mover_confirms_and_latches():
    assert _track_moving(MOTION_CONFIRM)["moving"] is True
    # Stops: still latched until MOTION_RELEASE_FRAMES frames without evidence.
    # The window keeps showing travel for ~WINDOW_SIZE frames after stopping.
    assert _track_moving(MOTION_CONFIRM, 10)["moving"] is True
    assert _track_moving(MOTION_CONFIRM,
                         WINDOW_SIZE + MOTION_RELEASE_FRAMES + 2)["moving"] is False


def test_static_blob_never_moving():
    tr = ObstacleTracker()
    rng = random.Random(2)
    for _ in range(60):
        x = 2.0 + rng.gauss(0, 0.06)
        out = tr.update([(0.0, 0.0, x)], [(x, rng.gauss(0, 0.06), x)], [False])
    assert next(iter(out.values()))["moving"] is False
