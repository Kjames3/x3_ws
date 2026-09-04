"""Offline regression tests for calibrated velocity-estimator projection."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from velocity_estimator import VelocityEstimator  # noqa: E402


def _estimator_without_model():
    estimator = VelocityEstimator.__new__(VelocityEstimator)
    estimator.detections_fn = None
    estimator._logged_missing_intrinsics = False
    return estimator


def test_scale_intrinsics_uses_actual_frame_dimensions():
    intr = (600.0, 620.0, 300.0, 200.0, 600, 400)
    assert VelocityEstimator._scale_intrinsics(intr, (200, 300)) == \
        (300.0, 310.0, 150.0, 100.0)


def test_raw_centroid_uses_distinct_fy_and_calibrated_principal_point():
    estimator = _estimator_without_model()
    depth = np.zeros((200, 300), dtype=np.float32)
    # A large, clean off-axis obstacle survives the 5x5 morphology and area gate.
    depth[80:160, 160:260] = 2.0
    intr = (300.0, 400.0, 120.0, 70.0, 300, 200)

    centroids = estimator._extract_depth_centroids(None, depth, intr)
    assert len(centroids) == 1, centroids
    x, y, z = centroids[0]
    # Downsampling maps the rectangle centroid to approximately (104.5, 59.5),
    # while intrinsics become fx=150, fy=200, cx=60, cy=35.
    assert np.isclose(z, 2.0)
    assert np.isclose(x, (104.5 - 60.0) * 2.0 / 150.0, atol=0.02)
    assert np.isclose(y, (59.5 - 35.0) * 2.0 / 200.0, atol=0.02)


def test_centroids_are_suppressed_until_intrinsics_are_available():
    estimator = _estimator_without_model()
    depth = np.ones((100, 100), dtype=np.float32)
    assert estimator._extract_depth_centroids(None, depth, None) == []


if __name__ == "__main__":
    tests = [(name, fn) for name, fn in sorted(globals().items())
             if name.startswith("test_") and callable(fn)]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:
            failures += 1
            print(f"  FAIL  {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
