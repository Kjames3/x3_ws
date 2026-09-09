"""ReplayBridge — a drop-in for ROS2Bridge that replays recorded fixtures.

Lets you work on `server_x3.py`, the GUI, the frontier explorer and the velocity
estimator with the robot powered off and no ROS installed. It exposes the same
public surface `server_x3.py` consumes from `ROS2Bridge` — the getters, `move`,
`stop`, `cleanup`, and the `_lock` / `_twist` / `_odom_stamp` attributes the
broadcast loop reaches into directly — but every value comes from `fixtures/`
instead of DDS.

    python3 scripts/make_fixtures.py       # once, to build fixtures/ from a bag
    python3 src/server_x3.py --replay

What it is NOT: a simulator. `move()` is recorded and echoed back through the
twist so the GUI shows the commanded velocity, but it does not integrate into the
replayed pose — the pose comes from the bag and will happily drive through walls
you thought you avoided. Use Gazebo (`--sim`) when you need the loop closed.

Fixtures are decoded numpy, not serialized ROS messages, so nothing here imports
rclpy. That is deliberate: the point is to run on a machine that has never had ROS.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FIXTURES = REPO_ROOT / "fixtures"
DEFAULT_MAP = REPO_ROOT / "src" / "yahboomcar_nav" / "maps" / "test1"

# Matches ROS2Bridge._scan_cb: the lidar is mounted backwards (yaw 180 deg) and
# 43.5 mm forward of base_link, and returns closer than 0.12 m are the chassis.
_LASER_X_OFFSET = 0.0435
_MIN_VALID_RANGE = 0.12
_MAX_OBSTACLE_RANGE = 1.0


def _load_npz(path: Path) -> dict | None:
    """Load one fixture file, unwrapping the `_meta_` scalars make_fixtures wrote."""
    if not path.exists():
        return None
    out = {}
    with np.load(path, allow_pickle=False) as z:
        for key in z.files:
            value = z[key]
            if key.startswith("_meta_"):
                out[key[len("_meta_"):]] = value.item()
            else:
                out[key] = value
    return out


class ReplayBridge:
    """Replays fixtures on a background thread at their recorded wall-clock rate."""

    def __init__(self, fixtures: Path | str = DEFAULT_FIXTURES,
                 loop: bool = True, rate: float = 1.0,
                 map_stem: Path | str = DEFAULT_MAP):
        self.fixtures = Path(fixtures)
        if not self.fixtures.is_dir():
            raise FileNotFoundError(
                f"no fixtures at {self.fixtures} — build them with:\n"
                f"    python3 scripts/make_fixtures.py"
            )
        self._loop = loop
        self._rate = max(rate, 1e-3)

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None

        # Same fields ROS2Bridge exposes; server_x3.py touches several directly.
        self._node = None                  # no rclpy node: Nav2/OAK publishing stay disabled
        self._latest_frame = None
        self._latest_depth = None
        self._latest_raw_depth = None
        self._latest_obstacles: list[tuple[float, float]] = []
        self._pose_m = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self._twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._voltage = 12.0
        self._occupancy_grid: dict | None = None
        self._odom_stamp = 0.0
        self._last_depth_write_time = 0.0
        self._commanded = {"vx": 0.0, "vy": 0.0, "wz": 0.0}

        manifest_path = self.fixtures / "manifest.json"
        self.manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        self._scan = _load_npz(self.fixtures / "scan.npz")
        self._odom = _load_npz(self.fixtures / "odom.npz")
        self._depth = _load_npz(self.fixtures / "depth.npz")

        if not any((self._scan, self._odom, self._depth)):
            raise FileNotFoundError(f"{self.fixtures} has no scan/odom/depth fixtures")

        self._occupancy_grid = self._load_map(Path(map_stem))

        logger.info(
            "ReplayBridge: bag=%s scan=%d odom=%d depth=%d map=%s",
            self.manifest.get("source_bag", "?"),
            0 if self._scan is None else len(self._scan["stamps_ns"]),
            0 if self._odom is None else len(self._odom["stamps_ns"]),
            0 if self._depth is None else len(self._depth["stamps_ns"]),
            "yes" if self._occupancy_grid else "no",
        )

    # -- map -------------------------------------------------------------

    @staticmethod
    def _load_map(stem: Path) -> dict | None:
        """Read a SLAM-saved .pgm/.yaml pair into the /map grid dict shape.

        Reimplements the PGM read rather than pulling in yaml/cv2, so this module
        keeps its "numpy and nothing else" property.
        """
        pgm, yml = stem.with_suffix(".pgm"), stem.with_suffix(".yaml")
        if not (pgm.exists() and yml.exists()):
            return None
        try:
            meta = {}
            for line in yml.read_text().splitlines():
                if ":" not in line or line.lstrip().startswith("#"):
                    continue
                key, _, value = line.partition(":")
                meta[key.strip()] = value.strip()
            origin = json.loads(meta.get("origin", "[0,0,0]").replace("'", '"'))
            resolution = float(meta.get("resolution", 0.05))
            negate = int(meta.get("negate", 0))
            occupied_th = float(meta.get("occupied_thresh", 0.65))
            free_th = float(meta.get("free_thresh", 0.196))

            data = pgm.read_bytes()
            if not data.startswith(b"P5"):
                logger.warning("ReplayBridge: %s is not a binary PGM (P5); skipping map", pgm)
                return None
            # Header: P5, width height, maxval — comments (#) may appear between.
            fields, pos = [], 2
            while len(fields) < 3:
                while pos < len(data) and data[pos:pos + 1].isspace():
                    pos += 1
                if data[pos:pos + 1] == b"#":
                    pos = data.index(b"\n", pos) + 1
                    continue
                start = pos
                while pos < len(data) and not data[pos:pos + 1].isspace():
                    pos += 1
                fields.append(int(data[start:pos]))
            width, height, maxval = fields
            pos += 1  # single whitespace byte after maxval, per the PGM spec
            dtype = np.uint8 if maxval < 256 else ">u2"
            pixels = np.frombuffer(data, dtype=dtype, count=width * height,
                                   offset=pos).reshape(height, width)

            # map_server semantics: high pixel = free, low = occupied (unless negated).
            occ = pixels.astype(np.float32) / maxval
            if not negate:
                occ = 1.0 - occ
            grid = np.full(occ.shape, -1, dtype=np.int8)
            grid[occ > occupied_th] = 100
            grid[occ < free_th] = 0
            # map_saver writes unknown as the canonical mid-grey (205 for 8-bit),
            # whose implied occupancy is (255-205)/255 = 0.196. These maps carry
            # free_thresh = 0.25, which is ABOVE that -- so thresholding alone
            # would relabel every unknown cell as free (134240 of 147456 in
            # test1.pgm). Unknown cells are precisely what frontier_explorer.py
            # searches for, so honour the sentinel explicitly and let it win.
            if maxval == 255:
                grid[pixels == 205] = -1
            # PGM rows run top-down; OccupancyGrid rows run bottom-up from the origin.
            grid = np.flipud(grid)
            return {
                "data": grid.reshape(-1),
                "width": width,
                "height": height,
                "resolution": resolution,
                "origin_x": float(origin[0]),
                "origin_y": float(origin[1]),
                "origin_yaw": float(origin[2]) if len(origin) > 2 else 0.0,
            }
        except Exception as exc:
            logger.warning("ReplayBridge: could not load map %s: %s", stem, exc)
            return None

    # -- playback --------------------------------------------------------

    def start(self):
        """Begin replaying. Idempotent."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, name="replay-bridge", daemon=True)
        self._thread.start()

    def _run(self):
        # One merged timeline of (offset_seconds, kind, index) so topics keep their
        # true relative ordering and rates instead of each free-running.
        events: list[tuple[float, str, int]] = []
        t0 = min(f["stamps_ns"][0] for f in (self._scan, self._odom, self._depth) if f is not None)
        for kind, fixture in (("scan", self._scan), ("odom", self._odom), ("depth", self._depth)):
            if fixture is None:
                continue
            for i, ts in enumerate(fixture["stamps_ns"]):
                events.append(((ts - t0) / 1e9, kind, i))
        events.sort()
        span = events[-1][0] if events else 0.0

        while not self._stop_evt.is_set():
            started = time.monotonic()
            for offset, kind, i in events:
                target = started + offset / self._rate
                delay = target - time.monotonic()
                if delay > 0 and self._stop_evt.wait(delay):
                    return
                self._apply(kind, i)
            if not self._loop:
                return
            # Hold the last frame for one inter-loop gap so the seam isn't a jump.
            if self._stop_evt.wait(max(0.0, (span / self._rate) * 0.02)):
                return

    def _apply(self, kind: str, i: int):
        if kind == "scan":
            self._apply_scan(i)
        elif kind == "odom":
            self._apply_odom(i)
        else:
            self._apply_depth(i)

    def _apply_scan(self, i: int):
        f = self._scan
        n = int(f["counts"][i])
        ranges = f["ranges"][i, :n]
        angles = f["angle_min"][i] + np.arange(n, dtype=np.float32) * f["angle_increment"][i]
        valid = (ranges > _MIN_VALID_RANGE) & (ranges < _MAX_OBSTACLE_RANGE)
        r, a = ranges[valid], angles[valid]
        # Same laser_link -> base_link transform as ROS2Bridge._scan_cb, vectorized.
        xs = -(r * np.cos(a)) + _LASER_X_OFFSET
        ys = -(r * np.sin(a))
        obstacles = list(zip(xs.tolist(), ys.tolist()))
        with self._lock:
            self._latest_obstacles = obstacles

    def _apply_odom(self, i: int):
        f = self._odom
        x, y, theta = f["pose_xytheta"][i]
        vx, vy, wz = f["twist_vxvywz"][i]
        self._odom_stamp = time.monotonic()
        with self._lock:
            self._pose_m = {"x": float(x), "y": float(y), "theta": float(theta)}
            self._twist = {"vx": float(vx), "vy": float(vy), "wz": float(wz)}

    def _apply_depth(self, i: int):
        f = self._depth
        meters = f["frames_mm"][i].astype(np.float32) / 1000.0
        self._last_depth_write_time = time.monotonic()
        with self._lock:
            self._latest_raw_depth = meters
            self._latest_depth = self._colourise(meters)

    @staticmethod
    def _colourise(meters: np.ndarray):
        """Match ROS2Bridge._depth_cb: clamp, invert, dynamic-range to BONE."""
        clean = np.where((meters <= 0.1) | np.isnan(meters), 5.0, meters)
        clean = np.clip(clean, 0.3, 5.0)
        lo, hi = float(clean.min()), float(clean.max())
        if hi > lo:
            norm = (255.0 * (1.0 - (clean - lo) / (hi - lo))).astype(np.uint8)
        else:
            norm = np.zeros(clean.shape, dtype=np.uint8)
        try:
            import cv2
            return cv2.applyColorMap(norm, cv2.COLORMAP_BONE)
        except ImportError:
            # cv2 is optional here so the bridge still runs on a bare-numpy box.
            return np.repeat(norm[:, :, None], 3, axis=2)

    # -- ROS2Bridge-compatible surface ------------------------------------

    def get_frame(self):
        """Always None: the bags carry depth but no RGB on /camera/image_raw."""
        with self._lock:
            return self._latest_frame

    def get_depth_frame(self):
        with self._lock:
            return self._latest_depth

    def get_raw_depth_frame(self):
        with self._lock:
            return self._latest_raw_depth

    def get_depth_frame_age(self) -> float:
        if self._last_depth_write_time == 0.0:
            return float("inf")
        return time.monotonic() - self._last_depth_write_time

    def get_wheel_velocities(self) -> tuple:
        L = 0.20
        with self._lock:
            t = dict(self._twist)
        vx, vy, wz = t["vx"], t["vy"], t["wz"]
        return (vx - vy - L * wz, vx + vy + L * wz,
                vx + vy - L * wz, vx - vy + L * wz)

    def get_pose_m(self) -> dict:
        with self._lock:
            return dict(self._pose_m)

    def get_pose_cm(self) -> dict:
        with self._lock:
            p = self._pose_m
            return {"x": p["x"] * 100.0, "y": p["y"] * 100.0, "theta": p["theta"]}

    def get_battery_voltage(self) -> float:
        with self._lock:
            return self._voltage

    def get_occupancy_grid(self) -> dict | None:
        with self._lock:
            return dict(self._occupancy_grid) if self._occupancy_grid else None

    def move(self, vx: float, vy: float, omega: float):
        """Record the command. Nothing moves — see the module docstring."""
        with self._lock:
            self._commanded = {"vx": vx, "vy": vy, "wz": omega}

    def get_commanded(self) -> dict:
        """Last velocity passed to move(); useful for asserting in tests."""
        with self._lock:
            return dict(self._commanded)

    def stop(self):
        self.move(0.0, 0.0, 0.0)

    def cleanup(self):
        self._stop_evt.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def publish_pedestrians(self, estimates):
        """No-op: there is no ROS graph to publish onto in replay mode."""
        return None
