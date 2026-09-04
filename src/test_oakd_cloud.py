"""
test_oakd_cloud.py — offline tests for the OAK-D depth -> PointCloud2 path.

Runs on a laptop: no device, no rclpy. Covers the projection geometry (the part
that silently produces a plausible-looking but wrong cloud if the intrinsics or
the stride indexing are off) and the intrinsics rescaling in OakDCamera.

    python3 src/test_oakd_cloud.py
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import oakd_cloud  # noqa: E402

FX, FY, CX, CY = 300.0, 310.0, 240.0, 320.0
H, W = 640, 480     # portrait, matching the spatial pipeline's setOutputSize


def _blank(fill=0.0):
    return np.full((H, W), fill, dtype=np.float32)


def test_roundtrip_projection():
    """A point placed at a known pixel must back-project to the point itself."""
    depth = _blank()
    u, v, z = 100, 200, 2.0
    depth[v, u] = z

    pts = oakd_cloud.depth_to_points(depth, FX, FY, CX, CY, stride=1)
    assert pts.shape == (1, 3), pts.shape

    expect = np.array([(u - CX) * z / FX, (v - CY) * z / FY, z], dtype=np.float32)
    assert np.allclose(pts[0], expect, atol=1e-5), (pts[0], expect)
    # Sanity on sign conventions: (100, 200) is left of and above the principal
    # point (240, 320), so in optical frame (x right, y down) both x and y are
    # negative. Getting y's sign wrong flips floor and ceiling, which is exactly
    # what would make min_obstacle_height filter the wrong half of the cloud.
    assert pts[0][0] < 0 and pts[0][1] < 0

    # And a pixel below the principal point must give y > 0.
    below = _blank()
    below[CY_I := 400, u] = z
    assert oakd_cloud.depth_to_points(below, FX, FY, CX, CY, stride=1)[0][1] > 0
    assert CY_I > CY


def test_stride_indexing_uses_full_res_pixel_coords():
    """With stride>1 the pixel index must still be the full-res u,v.

    Forgetting the *stride multiply here is the classic bug: the cloud comes out
    looking sane but compressed toward the optical axis by the stride factor.
    """
    depth = _blank()
    u, v, z = 120, 240, 3.0     # divisible by 4 so it survives the stride
    depth[v, u] = z

    pts = oakd_cloud.depth_to_points(depth, FX, FY, CX, CY, stride=4)
    assert pts.shape == (1, 3), pts.shape
    expect = np.array([(u - CX) * z / FX, (v - CY) * z / FY, z], dtype=np.float32)
    assert np.allclose(pts[0], expect, atol=1e-5), (pts[0], expect)


def test_range_gate_and_invalid_pixels():
    depth = _blank()
    depth[100, 100] = 0.0            # no return
    depth[104, 100] = np.nan         # failed stereo match
    depth[108, 100] = np.inf
    depth[112, 100] = 0.10           # nearer than z_min
    depth[116, 100] = 9.0            # farther than z_max
    depth[120, 100] = 1.5            # the only keeper

    pts = oakd_cloud.depth_to_points(depth, FX, FY, CX, CY, stride=4,
                                     z_min=0.30, z_max=4.0)
    assert pts.shape == (1, 3), pts.shape
    assert abs(float(pts[0][2]) - 1.5) < 1e-6
    assert np.isfinite(pts).all()


def test_empty_inputs():
    assert oakd_cloud.depth_to_points(None, FX, FY, CX, CY).shape == (0, 3)
    assert oakd_cloud.depth_to_points(_blank(), FX, FY, CX, CY).shape == (0, 3)
    # An all-invalid frame must yield an empty cloud, not a crash — the voxel
    # layer still wants the message so it can raytrace-clear.
    assert oakd_cloud.depth_to_points(_blank(np.nan), FX, FY, CX, CY).shape == (0, 3)


def test_max_points_cap():
    depth = _blank(2.0)     # every pixel valid
    cap = 500
    pts = oakd_cloud.depth_to_points(depth, FX, FY, CX, CY, stride=4,
                                     max_points=cap)
    assert pts.shape[0] == cap, pts.shape
    assert np.isfinite(pts).all()
    # The cap must subsample across the whole frame, not crop a corner.
    assert pts[:, 0].min() < 0 < pts[:, 0].max()
    assert pts[:, 1].min() < 0 < pts[:, 1].max()


def test_plane_at_constant_depth_is_planar():
    """A wall at constant Z must come back with constant z, and x/y spanning."""
    depth = _blank(2.5)
    pts = oakd_cloud.depth_to_points(depth, FX, FY, CX, CY, stride=8,
                                     max_points=0)
    assert pts.shape[0] > 100
    assert np.allclose(pts[:, 2], 2.5, atol=1e-5)
    # Horizontal half-extent at 2.5 m: (W/2)/fx * z on each side.
    assert abs(pts[:, 0].max() - (W - 1 - CX) * 2.5 / FX) < 0.1


def test_ray_grid_cache_is_bounded():
    for i in range(30):
        oakd_cloud.depth_to_points(_blank(1.0), FX + i, FY, CX, CY, stride=4,
                                   max_points=10)
    assert len(oakd_cloud._grid_cache) <= 9, len(oakd_cloud._grid_cache)


def test_scale_intrinsics_matching_size_is_noop():
    """The normal case: depth arrives at exactly the size intrinsics were read at."""
    intr = (FX, FY, CX, CY, W, H)
    assert oakd_cloud.scale_intrinsics(intr, (H, W)) == (FX, FY, CX, CY)


def test_scale_intrinsics_rescales_to_actual_frame():
    intr = (FX, FY, CX, CY, W, H)               # valid at 480x640

    # Half-size depth map -> every intrinsic halves.
    assert oakd_cloud.scale_intrinsics(intr, (H // 2, W // 2)) == \
        (FX / 2, FY / 2, CX / 2, CY / 2)

    # Non-uniform rescale must scale x and y independently.
    fx, fy, cx, cy = oakd_cloud.scale_intrinsics(intr, (H, W // 2))
    assert (fx, cx) == (FX / 2, CX / 2)
    assert (fy, cy) == (FY, CY)


def test_scale_intrinsics_none_before_device_up():
    """get_depth_intrinsics() returns None until calibration is read."""
    assert oakd_cloud.scale_intrinsics(None, (H, W)) is None


def test_driver_exposes_get_depth_intrinsics():
    """Guard the contract this module depends on."""
    from oakd_driver import OakDCamera
    assert callable(getattr(OakDCamera, "get_depth_intrinsics", None))


class _FakeCalibration:
    def __init__(self):
        self.calls = []

    def getCameraIntrinsics(self, socket, width, height):
        self.calls.append((socket, width, height))
        return [[301.0, 0.0, 241.0],
                [0.0, 311.0, 201.0],
                [0.0, 0.0, 1.0]]


class _FakeDevice:
    def __init__(self, calibration=None, error=None):
        self.calibration = calibration
        self.error = error

    def readCalibration(self):
        if self.error is not None:
            raise self.error
        return self.calibration


def _patch_fake_dai(oakd_driver):
    old_dai = oakd_driver.dai
    sockets = SimpleNamespace(CAM_A="CAM_A", CAM_B="CAM_B", CAM_C="CAM_C")
    oakd_driver.dai = SimpleNamespace(CameraBoardSocket=sockets)
    return old_dai


def test_driver_reads_cam_a_intrinsics_at_spatial_depth_size():
    import oakd_driver

    old_dai = _patch_fake_dai(oakd_driver)
    try:
        camera = oakd_driver.OakDCamera(sim_mode=True)
        calibration = _FakeCalibration()
        assert camera.get_depth_intrinsics() is None
        camera._read_intrinsics(_FakeDevice(calibration), with_spatial=True)

        assert calibration.calls == [("CAM_A", 480, 640)]
        assert camera.get_depth_intrinsics() == (301.0, 311.0, 241.0, 201.0,
                                                  480, 640)
        assert (camera._fx, camera._fy, camera._cx, camera._cy) == \
            (301.0, 311.0, 241.0, 201.0)
    finally:
        oakd_driver.dai = old_dai


def test_driver_reads_selected_stereo_camera_at_640x400():
    import oakd_driver

    old_dai = _patch_fake_dai(oakd_driver)
    try:
        for align_left, expected_socket in ((True, "CAM_B"), (False, "CAM_C")):
            camera = oakd_driver.OakDCamera(sim_mode=True,
                                             align_depth_to_left=align_left)
            calibration = _FakeCalibration()
            camera._read_intrinsics(_FakeDevice(calibration), with_spatial=False)
            assert calibration.calls == [(expected_socket, 640, 400)]
            assert camera.get_depth_intrinsics() == \
                (301.0, 311.0, 241.0, 201.0, 640, 400)
    finally:
        oakd_driver.dai = old_dai


def test_driver_clears_stale_intrinsics_when_calibration_fails():
    import oakd_driver

    old_dai = _patch_fake_dai(oakd_driver)
    try:
        camera = oakd_driver.OakDCamera(sim_mode=True)
        camera._read_intrinsics(_FakeDevice(_FakeCalibration()), with_spatial=True)
        assert camera.get_depth_intrinsics() is not None

        camera._read_intrinsics(_FakeDevice(error=RuntimeError("device lost")),
                                with_spatial=False)
        assert camera.get_depth_intrinsics() is None
    finally:
        oakd_driver.dai = old_dai


def test_driver_oak_extrinsics_match_urdf():
    """Prevent the driver's base-frame projection drifting from robot geometry."""
    from oakd_driver import OAK_MOUNT_X, OAK_MOUNT_Z

    urdf = Path(__file__).parent / "yahboomcar_description" / "urdf" / "yahboomcar_X3.urdf"
    root = ET.parse(urdf).getroot()
    joint = next(j for j in root.findall("joint") if j.get("name") == "oak_center_joint")
    x, _y, z = map(float, joint.find("origin").get("xyz").split())

    assert np.isclose(OAK_MOUNT_X, x)
    assert np.isclose(OAK_MOUNT_Z, z)


def test_oak_mount_uses_measured_x3plus_position():
    from oakd_driver import OAK_MOUNT_X, OAK_MOUNT_Z

    assert np.isclose(OAK_MOUNT_X, 0.107315)
    assert np.isclose(OAK_MOUNT_Z, 0.134)


def test_scaled_intrinsics_project_consistently():
    """Scaling then projecting a half-res frame must land on the same 3D point.

    A point at the same *relative* image position in a half-resolution depth map
    must back-project to the same metric coordinates — this is the property that
    makes the rescale safe rather than merely plausible.
    """
    intr = (FX, FY, CX, CY, W, H)
    z = 2.0

    full = _blank()
    full[300, 200] = z
    p_full = oakd_cloud.depth_to_points(
        full, *oakd_cloud.scale_intrinsics(intr, (H, W)), stride=1)

    half = np.zeros((H // 2, W // 2), dtype=np.float32)
    half[150, 100] = z
    p_half = oakd_cloud.depth_to_points(
        half, *oakd_cloud.scale_intrinsics(intr, (H // 2, W // 2)), stride=1)

    assert np.allclose(p_full[0], p_half[0], atol=1e-5), (p_full[0], p_half[0])


if __name__ == "__main__":
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in fns:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed += 1
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    sys.exit(1 if failed else 0)
