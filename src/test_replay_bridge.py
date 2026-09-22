#!/usr/bin/env python3
"""Offline smoke test for ReplayBridge — no robot, no ROS, no DDS.

    python3 src/test_replay_bridge.py

Exits non-zero on the first failure. Deliberately a plain script (like the other
test_*.py in src/) rather than pytest, so it runs with nothing installed but numpy.
"""

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from replay_bridge import ReplayBridge  # noqa: E402

FAILURES = []


def check(name: str, condition: bool, detail: str = ""):
    status = "ok  " if condition else "FAIL"
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def main() -> int:
    fixtures = Path(__file__).resolve().parent.parent / "fixtures"
    if not fixtures.is_dir():
        print(f"no fixtures at {fixtures}; run: python3 scripts/make_fixtures.py")
        return 2

    print("ReplayBridge offline smoke test")
    bridge = ReplayBridge(fixtures)

    print("\nstatic surface (before playback starts)")
    for method in ("get_frame", "get_depth_frame", "get_raw_depth_frame",
                   "get_depth_frame_age", "get_wheel_velocities", "get_pose_m",
                   "get_pose_cm", "get_battery_voltage", "get_occupancy_grid",
                   "move", "stop", "cleanup", "publish_pedestrians"):
        check(f"has {method}()", callable(getattr(bridge, method, None)))
    check("get_depth_frame_age() is inf before any frame",
          bridge.get_depth_frame_age() == float("inf"))

    grid = bridge.get_occupancy_grid()
    check("occupancy grid loaded from the .pgm/.yaml pair", grid is not None)
    if grid:
        check("grid data length == width * height",
              len(grid["data"]) == grid["width"] * grid["height"],
              f"{grid['width']}x{grid['height']}")
        values = set(np.unique(grid["data"]).tolist())
        check("grid holds only OccupancyGrid values {-1, 0, 100}",
              values <= {-1, 0, 100}, str(sorted(values)))
        check("grid has some free space", int((grid["data"] == 0).sum()) > 0)
        check("grid has some occupied cells", int((grid["data"] == 100).sum()) > 0)
        # Without unknown cells there are no frontiers, so frontier_explorer.py
        # would silently find nothing to explore — the whole point of the fixture.
        check("grid has unknown cells (frontier explorer needs them)",
              int((grid["data"] == -1).sum()) > 0,
              f"{int((grid['data'] == -1).sum())} unknown")
        check("resolution is plausible", 0.001 < grid["resolution"] < 1.0,
              f"{grid['resolution']} m/cell")

    print("\nreal consumer: frontier_explorer against the replayed grid")
    if grid:
        try:
            from frontier_explorer import _find_frontier_centroids
        except Exception as exc:                       # numpy-only box, or scipy missing
            print(f"  [skip] frontier_explorer not importable — {exc}")
        else:
            centroids = _find_frontier_centroids(
                grid["data"], grid["width"], grid["height"],
                grid["resolution"], grid["origin_x"], grid["origin_y"])
            check("frontier explorer finds frontiers offline", len(centroids) > 0,
                  f"{len(centroids)} centroids")
            check("centroids are finite world coordinates",
                  all(np.isfinite(x) and np.isfinite(y) for x, y in centroids))

    print("\nplayback")
    bridge.start()
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if (bridge.get_raw_depth_frame() is not None
                and bridge._latest_obstacles
                and bridge._odom_stamp):
            break
        time.sleep(0.05)

    depth = bridge.get_raw_depth_frame()
    check("depth frame arrived", depth is not None)
    if depth is not None:
        check("depth is float32 metres", depth.dtype == np.float32, str(depth.shape))
        finite = depth[np.isfinite(depth) & (depth > 0)]
        check("depth values are in a sane metre range",
              finite.size > 0 and float(finite.max()) < 20.0,
              f"max {float(finite.max()):.2f} m" if finite.size else "no valid pixels")
        check("colourised depth is BGR uint8",
              bridge.get_depth_frame() is not None
              and bridge.get_depth_frame().dtype == np.uint8)

    obstacles = list(bridge._latest_obstacles)
    check("scan produced obstacles", len(obstacles) > 0, f"{len(obstacles)} points")
    if obstacles:
        radii = [float(np.hypot(x, y)) for x, y in obstacles]
        check("all obstacles within the 1.0 m gate", max(radii) < 1.05,
              f"max {max(radii):.2f} m")

    check("odom timestamp advanced", bridge._odom_stamp > 0)
    pose = bridge.get_pose_m()
    check("pose is finite", all(np.isfinite(v) for v in pose.values()), str(pose))
    check("pose_cm is pose_m * 100",
          abs(bridge.get_pose_cm()["x"] - pose["x"] * 100.0) < 1e-6)

    print("\nmotion is recorded but not simulated")
    bridge.move(0.3, 0.0, 0.5)
    check("move() records the command", bridge.get_commanded()["vx"] == 0.3)
    bridge.stop()
    check("stop() zeroes the command", bridge.get_commanded()["vx"] == 0.0)

    print("\nkinematics")
    with bridge._lock:
        bridge._twist = {"vx": 1.0, "vy": 0.0, "wz": 0.0}
    check("pure forward twist drives all four wheels equally",
          bridge.get_wheel_velocities() == (1.0, 1.0, 1.0, 1.0))
    with bridge._lock:
        bridge._twist = {"vx": 0.0, "vy": 0.0, "wz": 1.0}
    fl, fr, rl, rr = bridge.get_wheel_velocities()
    check("pure rotation counter-rotates left and right sides",
          fl < 0 < fr and rl < 0 < rr, f"({fl}, {fr}, {rl}, {rr})")

    bridge.cleanup()
    check("cleanup() stops the playback thread",
          bridge._thread is not None and not bridge._thread.is_alive())

    print()
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED: {', '.join(FAILURES)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
