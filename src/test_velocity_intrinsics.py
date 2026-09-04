"""Offline regression tests for calibrated velocity-estimator projection."""

import os
import sys
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from velocity_estimator import VelocityEstimator  # noqa: E402


def _estimator_without_model():
    estimator = VelocityEstimator.__new__(VelocityEstimator)
    estimator.detections_fn = None
    estimator._logged_missing_intrinsics = False
    estimator._height_ray_cache = {}
    estimator.camera_height_m = 0.210
    estimator.camera_pitch_rad = 0.0
    estimator.min_obstacle_height_m = 0.15
    estimator.max_obstacle_height_m = 2.0
    return estimator


def test_scale_intrinsics_uses_actual_frame_dimensions():
    intr = (600.0, 620.0, 300.0, 200.0, 600, 400)
    assert VelocityEstimator._scale_intrinsics(intr, (200, 300)) == \
        (300.0, 310.0, 150.0, 100.0)


def test_raw_centroid_uses_distinct_fy_and_calibrated_principal_point():
    estimator = _estimator_without_model()
    depth = np.zeros((200, 300), dtype=np.float32)
    # A large, clean off-axis obstacle survives the 5x5 morphology and area gate.
    depth[10:70, 160:260] = 2.0
    intr = (300.0, 400.0, 120.0, 70.0, 300, 200)

    centroids = estimator._extract_depth_centroids(None, depth, intr)
    assert len(centroids) == 1, centroids
    x, y, z = centroids[0]
    # Downsampling maps the rectangle centroid to approximately (104.5, 19.5),
    # while intrinsics become fx=150, fy=200, cx=60, cy=35.
    assert np.isclose(z, 2.0)
    assert np.isclose(x, (104.5 - 60.0) * 2.0 / 150.0, atol=0.02)
    assert np.isclose(y, (19.5 - 35.0) * 2.0 / 200.0, atol=0.02)


def test_centroids_are_suppressed_until_intrinsics_are_available():
    estimator = _estimator_without_model()
    depth = np.ones((100, 100), dtype=np.float32)
    assert estimator._extract_depth_centroids(None, depth, None) == []


def test_height_mask_rejects_level_floor_before_contouring():
    estimator = _estimator_without_model()
    height, width = 320, 240
    fy, cy = 300.0, 160.0
    depth = np.zeros((height, width), dtype=np.float32)
    rows = np.arange(height, dtype=np.float32)
    below_axis = rows > cy
    # Exact depth of a horizontal floor 0.21 m below a level optical centre.
    depth[below_axis, :] = (estimator.camera_height_m * fy /
                            (rows[below_axis] - cy))[:, None]

    valid_range = (depth >= 0.5) & (depth <= 4.0)
    keep = estimator._height_band_mask(depth, fy, cy)
    assert not np.any(keep & valid_range)


def test_ground_filter_keeps_person_pixels_while_removing_floor():
    estimator = _estimator_without_model()
    depth = np.zeros((200, 300), dtype=np.float32)
    fy, cy = 200.0, 100.0
    rows = np.arange(200, dtype=np.float32)
    floor_rows = rows > cy
    depth[floor_rows, :] = (estimator.camera_height_m * fy /
                            (rows[floor_rows] - cy))[:, None]
    # A torso at 1 m and above the optical axis has positive floor height.
    depth[30:90, 110:190] = 1.0

    keep = estimator._height_band_mask(depth, fy, cy)
    assert keep[60, 150]
    assert not keep[150, 150]


def test_ground_config_matches_measured_height_and_urdf():
    src_dir = Path(__file__).parent
    with open(src_dir.parent / "config" / "camera_ground_plane.json") as stream:
        config = json.load(stream)
    root = ET.parse(src_dir / "yahboomcar_description" / "urdf" /
                    "yahboomcar_X3.urdf").getroot()
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    base_z = float(joints["base_joint"].find("origin").get("xyz").split()[2])
    oak_z = float(joints["oak_center_joint"].find("origin").get("xyz").split()[2])
    oak_rpy = tuple(map(float, joints["oak_center_joint"].find("origin")
                        .get("rpy").split()))

    assert np.isclose(config["camera_height_m"], 0.210)
    assert np.isclose(config["camera_height_m"], base_z + oak_z)
    assert oak_rpy == (0.0, 0.0, 0.0)
    assert np.isclose(config["camera_pitch_deg"], 0.0)


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
