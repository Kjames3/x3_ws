"""Tests for the offline model-comparison replay chain."""
import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from score_velocity_models import load_model, replay  # noqa: E402


def _capture(rows, window_size=10, infer_hz=10):
    return {"window_size": window_size, "infer_hz": infer_hz, "rows": rows}


def _row(frame, tid, status="ok", visible=10, feats=None):
    return {"frame": frame, "t": frame / 10.0, "tid": tid, "z": 1.5,
            "visible_count": visible, "status": status,
            "feats": feats if feats is not None else [0.1] * 40}


def test_all_four_models_load_with_their_own_scaler():
    for name in ("v1", "v2", "v3", "compress25"):
        model, scaler = load_model(name)
        assert scaler["x_mean"].shape == (40,)
        assert scaler["y_mean"].shape == (2,)


def test_v1_scaler_convention_differs_from_the_others():
    # v1's scaler was fitted on ABSOLUTE rel_x/rel_y; the deployed feature
    # builder emits translation-normalized values whose frame-0 entries are
    # identically zero. v2/v3/compress25 encode that with mean 0, scale 1.
    v1 = load_model("v1")[1]
    assert not np.allclose(v1["x_mean"][:2], 0.0)
    for name in ("v2", "v3", "compress25"):
        sc = load_model(name)[1]
        assert np.allclose(sc["x_mean"][:4], 0.0)
        assert np.allclose(sc["x_scale"][:4], 1.0)


def test_gated_tracks_contribute_zero_to_the_frame_max():
    # A frame whose only track is gated must score 0.0, not be dropped -- the
    # live max_obs_speed counts those frames.
    rows = [_row(0, 1, status="gated_range", feats=None),
            _row(1, 1, status="ok")]
    res = replay(_capture(rows), "v1", 10, 10)
    assert res["frame_max"][0] == 0.0
    assert res["frame_max"][1] > 0.0


def test_confidence_multiplier_scales_speed_down():
    # conf = visible_count / WINDOW_SIZE, so a half-visible track reads half
    # speed. This is a pipeline-side under-read with no model involvement.
    full = replay(_capture([_row(0, 1, visible=10)]), "v1", 10, 10)
    half = replay(_capture([_row(0, 1, visible=5)]), "v1", 10, 10)
    assert np.isclose(half["conf"][0], full["conf"][0] * 0.5, rtol=1e-5)
    # The raw model output is identical; only the pipeline differs.
    assert np.isclose(half["raw"][0], full["raw"][0], rtol=1e-6)


def test_acceleration_clamp_is_per_component_not_per_speed():
    # The deployed code clamps vx and vy INDEPENDENTLY and then recomputes
    # speed, so the speed itself can move by up to sqrt(2)*max_delta in one
    # frame. A scalar-speed clamp would wrongly cap it at max_delta and
    # under-report every acceleration.
    rows = [_row(0, 1, feats=[0.0] * 40), _row(1, 1)]
    res = replay(_capture(rows), "v1", 10, 10)
    max_delta = 3.0 / 10
    step = abs(res["frame_max"][1] - res["frame_max"][0])
    assert step <= math.sqrt(2) * max_delta + 1e-6


def test_gated_track_resets_the_clamp_history_to_zero():
    # A track that is gated enters the clamp history at vx=vy=0, so on the
    # frame it returns it can only ramp back to max_delta per component.
    rows = [_row(0, 1, status="gated_range", feats=None), _row(1, 1)]
    res = replay(_capture(rows), "v1", 10, 10)
    assert res["frame_max"][0] == 0.0
    assert res["frame_max"][1] <= math.sqrt(2) * (3.0 / 10) + 1e-6


def test_capture_without_ungated_frames_is_an_error():
    rows = [_row(0, 1, status="gated_stopped", feats=None)]
    with pytest.raises(SystemExit):
        replay(_capture(rows), "v1", 10, 10)
