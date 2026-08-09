"""
test_oakd_cloud.py — offline tests for the OAK-D depth -> PointCloud2 path.

Runs on a laptop: no device, no rclpy. Covers the projection geometry (the part
that silently produces a plausible-looking but wrong cloud if the intrinsics or
the stride indexing are off) and the intrinsics rescaling in OakDCamera.

    python3 src/test_oakd_cloud.py
"""

import os
import sys

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


class _FakeOak:
    """Minimal stand-in exercising OakDCamera.get_intrinsics' rescaling."""

    from oakd_driver import OakDCamera
    get_intrinsics = OakDCamera.get_intrinsics

    def __init__(self, size):
        self._fx, self._fy, self._cx, self._cy = FX, FY, CX, CY
        self._intr_size = size


def test_get_intrinsics_rescales_to_actual_frame():
    oak = _FakeOak((W, H))                      # intrinsics valid at 480x640
    assert oak.get_intrinsics((H, W)) == (FX, FY, CX, CY)

    # Half-size depth map -> every intrinsic halves.
    fx, fy, cx, cy = oak.get_intrinsics((H // 2, W // 2))
    assert (fx, fy, cx, cy) == (FX / 2, FY / 2, CX / 2, CY / 2)

    # Non-uniform rescale must scale x and y independently.
    fx, fy, cx, cy = oak.get_intrinsics((H, W // 2))
    assert (fx, cx) == (FX / 2, CX / 2)
    assert (fy, cy) == (FY, CY)


def test_get_intrinsics_none_before_device_up():
    oak = _FakeOak((W, H))
    oak._fx = None
    assert oak.get_intrinsics((H, W)) is None


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
