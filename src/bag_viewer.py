#!/usr/bin/env python3
"""
bag_viewer.py — browse and scrub rosbag2 recordings from the X3 robot.

Reads rosbag2 sqlite3 bags directly (no ROS graph, no `ros2 bag play`), builds a
timestamp index per topic, and lets you scrub anywhere in the bag instantly, in
either direction. Depth/RGB images, laser scans and odometry are rendered; every
other topic is shown as text.

Usage
-----
    python3 src/bag_viewer.py                       # GUI, scans the default dirs
    python3 src/bag_viewer.py <dir-or-bag> [...]    # GUI, scans the given paths
    python3 src/bag_viewer.py --list                # text: one line per bag
    python3 src/bag_viewer.py --info <bag>          # text: topics/sensors in one bag

Keys (GUI)
----------
    Space            play / pause
    Right / Left     step one message of the sync topic
    Shift+arrows     step +/- 1 s
    Ctrl+arrows      step +/- 10 s
    Home / End       jump to start / end of bag
    n / p            next / previous bag        (also PageDown / PageUp)
    i                cycle which image topic is displayed
    s                cycle the sync topic (what Right/Left steps through)
    + / -            playback speed up / down
    o                toggle the odometry trail
    d                toggle depth colour range (auto percentile <-> fixed 0.3-6 m)
    r                rescan the bag directories
    ? or h           show this help
    q or Esc         quit

Notes on these bags
-------------------
Bags recorded by record_bag.sh use `--compression-mode file --compression-format
zstd`, so each bag dir holds a `*.db3.zstd`. This viewer never decompresses in
place: if only the .zstd exists it is expanded into a cache dir under
~/.cache/x3_bag_viewer/. Bags whose .db3 was truncated (short transfer) are
auto-repaired by zero-padding to the page count in the sqlite header, which
recovers every message that made it to disk.
"""

from __future__ import annotations

import argparse
import bisect
import glob
import os
import shutil
import sqlite3
import struct
import sys
import time
from collections import OrderedDict
from datetime import datetime

import numpy as np

try:
    import yaml
except ImportError:  # metadata.yaml is optional — sqlite is the source of truth
    yaml = None

try:
    # Load PyQt5's Qt libraries first so they win over the copies opencv-python
    # bundles — with cv2 first, the process ends up with two Qt5 runtimes.
    import PyQt5.QtCore  # noqa: F401
except ImportError:
    pass

try:
    # opencv-python bundles its own (incompatible) Qt5 and points
    # QT_QPA_PLATFORM_PLUGIN_PATH at it on import, which makes PyQt5 fail with
    # 'Could not load the Qt platform plugin "xcb"'. Undo that so PyQt5 uses its
    # own plugins; a path the user set deliberately is left alone.
    _qt_plugin_path = os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH")
    import cv2
    if os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH") != _qt_plugin_path:
        if _qt_plugin_path is None:
            os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
        else:
            os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _qt_plugin_path
except ImportError:
    cv2 = None

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

DEFAULT_ROOTS = [
    "~/EE_244_Final_Project/bags",
    "~/bags",
    "./bags",
]

CACHE_DIR = os.path.expanduser("~/.cache/x3_bag_viewer")

# Human-readable names for the sensors behind the topics this robot records.
SENSOR_NAMES = {
    "/scan": "YDLidar X3 — 2D laser scan",
    "/odom": "EKF-fused odometry (robot_localization)",
    "/odom_raw": "Wheel-encoder dead-reckoning odometry",
    "/tf": "Coordinate frame transforms (dynamic)",
    "/tf_static": "Coordinate frame transforms (static)",
    "/imu/data": "Madgwick-filtered IMU",
    "/cmd_vel": "Velocity commands",
    "/voltage": "Battery voltage",
    "/map": "SLAM occupancy grid",
    "/camera/image_raw": "Orbbec Astra Pro — RGB",
    "/camera/depth/image_raw": "Orbbec Astra Pro — depth (16UC1, mm)",
    "/oak/depth/image_raw": "OAK-D Lite — on-device stereo depth (16UC1, mm)",
    "/oak/depth/camera_info": "OAK-D Lite — CAM_A intrinsics",
    "/oak/imu": "OAK-D Lite — 6-axis IMU",
    "/oak/detections": "OAK-D Lite — on-device YOLO spatial detections",
    "/oak/left/image_raw": "OAK-D Lite — left mono camera",
    "/oak/right/image_raw": "OAK-D Lite — right mono camera",
}

IMAGE_TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage")


def sensor_label(topic: str, type_str: str) -> str:
    if topic in SENSOR_NAMES:
        return SENSOR_NAMES[topic]
    return type_str.split("/")[-1]


def human_size(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return f"{n:.1f} {unit}" if unit != "B" else f"{n:.0f} B"
        n /= 1024
    return f"{n:.1f} TB"


# ---------------------------------------------------------------------------
# Bag discovery
# ---------------------------------------------------------------------------

def _bag_files(path: str):
    """Return (db3 files, zstd files) directly inside `path`."""
    db3 = sorted(f for f in glob.glob(os.path.join(path, "*.db3")))
    zst = sorted(f for f in glob.glob(os.path.join(path, "*.db3.zstd")))
    return db3, zst


def is_bag_dir(path: str) -> bool:
    if not os.path.isdir(path):
        return False
    db3, zst = _bag_files(path)
    return bool(db3 or zst)


def find_bags(roots) -> list:
    """Find every rosbag2 directory under `roots` (a bag dir may itself nest bags)."""
    found, seen = [], set()
    for root in roots:
        root = os.path.abspath(os.path.expanduser(root))
        if not os.path.exists(root):
            continue
        if os.path.isfile(root):  # a .db3 handed to us directly
            root = os.path.dirname(root)
        if is_bag_dir(root) and root not in seen:
            seen.add(root)
            found.append(root)
        for dirpath, dirnames, _ in os.walk(root):
            dirnames.sort()
            if dirpath in seen or not is_bag_dir(dirpath):
                continue
            seen.add(dirpath)
            found.append(dirpath)
    found.sort(key=lambda p: os.path.basename(p.rstrip("/")))
    return found


class BagSummary:
    """Cheap, metadata.yaml-based summary used to populate the bag list."""

    def __init__(self, path: str):
        self.path = path
        self.name = os.path.basename(path.rstrip("/")) or path
        self.message_count = None
        self.duration = None
        self.start_ns = None
        self.compression = ""
        self.storage = "sqlite3"
        self.topics = []  # (name, type, count)
        self.error = ""
        self.size = 0
        for f in glob.glob(os.path.join(path, "*")):
            if os.path.isfile(f):
                try:
                    self.size += os.path.getsize(f)
                except OSError:
                    pass
        self._read_metadata()

    def _read_metadata(self):
        meta = os.path.join(self.path, "metadata.yaml")
        if yaml is None or not os.path.exists(meta) or os.path.getsize(meta) == 0:
            return
        try:
            with open(meta) as fh:
                doc = yaml.safe_load(fh)
            info = (doc or {}).get("rosbag2_bagfile_information")
            if not info:
                return
            self.message_count = info.get("message_count")
            self.duration = info.get("duration", {}).get("nanoseconds", 0) / 1e9
            self.start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch")
            if self.start_ns == 2 ** 63 - 1:
                self.start_ns = None
            self.storage = info.get("storage_identifier", "sqlite3")
            fmt = info.get("compression_format") or ""
            mode = info.get("compression_mode") or ""
            self.compression = f"{fmt}/{mode}".strip("/") or "none"
            for t in info.get("topics_with_message_count") or []:
                tm = t.get("topic_metadata", {})
                self.topics.append((tm.get("name", "?"), tm.get("type", "?"),
                                    t.get("message_count", 0)))
        except Exception as exc:  # a half-written metadata.yaml must not hide the bag
            self.error = f"metadata.yaml unreadable: {exc}"

    @property
    def is_empty(self) -> bool:
        return self.message_count == 0

    def list_line(self) -> str:
        when = ("-" if self.start_ns is None else
                datetime.fromtimestamp(self.start_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S"))
        msgs = "?" if self.message_count is None else str(self.message_count)
        dur = "?" if self.duration is None else f"{self.duration:.1f}s"
        return (f"{self.name:<34} {when}  {dur:>8}  {msgs:>7} msgs  "
                f"{human_size(self.size):>9}  {self.compression}")

    def label(self, display_name=None) -> str:
        dur = "?" if self.duration is None else f"{self.duration:.0f}s"
        msgs = "?" if self.message_count is None else str(self.message_count)
        tag = "  (empty)" if self.is_empty else ""
        name = display_name or self.name
        return f"{name}\n   {dur} · {msgs} msgs · {human_size(self.size)}{tag}"


# ---------------------------------------------------------------------------
# Bag reading — direct sqlite access with a per-topic timestamp index
# ---------------------------------------------------------------------------

def _repair_truncated(db_path: str) -> str:
    """A short .db3 (interrupted copy) opens fine once padded to its header size.

    Returns the path to use — the original if padding worked in place, otherwise a
    padded copy in the cache dir. Padding only appends zero bytes, so no recorded
    data is ever overwritten.
    """
    try:
        size = os.path.getsize(db_path)
        with open(db_path, "rb") as fh:
            hdr = fh.read(100)
        if len(hdr) < 100 or hdr[:15] != b"SQLite format 3":
            return db_path
        page_size = struct.unpack(">H", hdr[16:18])[0]
        page_size = 65536 if page_size == 1 else page_size
        want = page_size * struct.unpack(">I", hdr[28:32])[0]
        if page_size <= 0 or want <= size:
            return db_path
    except OSError:
        return db_path

    target = db_path
    if not os.access(db_path, os.W_OK):
        os.makedirs(CACHE_DIR, exist_ok=True)
        target = os.path.join(CACHE_DIR, os.path.basename(db_path))
        if not os.path.exists(target) or os.path.getsize(target) != want:
            shutil.copy2(db_path, target)
    with open(target, "r+b") as fh:
        fh.seek(want - 1)
        fh.write(b"\x00")
    return target


def _decompress(zstd_path: str) -> str:
    """Expand a .db3.zstd into the cache dir (never next to the original)."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    out = os.path.join(CACHE_DIR, os.path.basename(zstd_path)[:-len(".zstd")])
    if os.path.exists(out) and os.path.getsize(out) > 0:
        return out
    try:
        import zstandard  # noqa: F401  (python-zstandard, if installed)
        import zstandard as zstd
        with open(zstd_path, "rb") as src, open(out, "wb") as dst:
            zstd.ZstdDecompressor().copy_stream(src, dst)
    except ImportError:
        if shutil.which("zstd") is None:
            raise RuntimeError(
                f"{os.path.basename(zstd_path)} is compressed and no decompressor is "
                "available (install python3-zstandard or the zstd CLI)")
        os.system(f'zstd -d -f -q -o "{out}" "{zstd_path}"')
    if not os.path.exists(out) or os.path.getsize(out) == 0:
        raise RuntimeError(f"decompression of {os.path.basename(zstd_path)} produced nothing")
    return out


class BagIndex:
    """Random-access reader over one rosbag2 bag directory."""

    def __init__(self, path: str, warn=None):
        self.path = path
        self.name = os.path.basename(path.rstrip("/")) or path
        self.summary = BagSummary(path)
        self.warnings = []
        self._warn = warn or self.warnings.append

        self.conns = []
        self.topic_type = {}
        self.index = {}          # topic -> (ts list, (conn_idx, rowid) list)
        self._msg_cls = {}
        self._cache = OrderedDict()
        self._trail_cache = {}

        self._open_files()
        self._build_index()

        all_ts = [ts for ts_list, _ in self.index.values() for ts in (ts_list[:1] + ts_list[-1:])]
        self.t0 = min(all_ts) if all_ts else 0
        self.t1 = max(all_ts) if all_ts else 0

    # -- opening ----------------------------------------------------------
    def _open_files(self):
        db3, zst = _bag_files(self.path)
        usable = []
        for f in db3:
            if os.path.getsize(f) > 0:
                usable.append(f)
        if not usable:
            for z in zst:
                if os.path.getsize(z) == 0:
                    continue
                try:
                    usable.append(_decompress(z))
                except Exception as exc:
                    self._warn(str(exc))
        if not usable:
            raise RuntimeError("no readable .db3 in this bag (all files are empty)")

        for f in usable:
            try:
                conn = self._connect(f)
            except sqlite3.DatabaseError:
                repaired = _repair_truncated(f)
                try:
                    conn = self._connect(repaired)
                    self._warn(f"{os.path.basename(f)} was truncated — repaired by "
                               "padding to the size in its sqlite header")
                except sqlite3.DatabaseError as exc:
                    self._warn(f"{os.path.basename(f)} is unreadable: {exc}")
                    continue
            self.conns.append(conn)
        if not self.conns:
            raise RuntimeError("every storage file in this bag failed to open")

    @staticmethod
    def _connect(path: str):
        conn = sqlite3.connect(f"file:{path}?mode=ro", uri=True, check_same_thread=False)
        conn.execute("select count(*) from topics").fetchone()  # fail fast if corrupt
        return conn

    def _build_index(self):
        for ci, conn in enumerate(self.conns):
            id_topic = {}
            for tid, name, type_str in conn.execute("select id,name,type from topics"):
                id_topic[tid] = name
                self.topic_type.setdefault(name, type_str)
                self.index.setdefault(name, ([], []))
            try:
                rows = conn.execute("select id,topic_id,timestamp from messages "
                                    "order by timestamp").fetchall()
            except sqlite3.DatabaseError as exc:
                self._warn(f"message index partially unreadable: {exc}")
                rows = []
            for rid, tid, ts in rows:
                name = id_topic.get(tid)
                if name is None:
                    continue
                ts_list, loc_list = self.index[name]
                ts_list.append(ts)
                loc_list.append((ci, rid))
        # A bag written out of order would break bisect; sort defensively.
        for name, (ts_list, loc_list) in self.index.items():
            if any(b < a for a, b in zip(ts_list, ts_list[1:])):
                pairs = sorted(zip(ts_list, loc_list))
                self.index[name] = ([p[0] for p in pairs], [p[1] for p in pairs])

    # -- queries ----------------------------------------------------------
    @property
    def duration(self) -> float:
        return (self.t1 - self.t0) / 1e9

    def topics(self) -> list:
        return sorted(self.index.keys())

    def count(self, topic: str) -> int:
        return len(self.index.get(topic, ([], []))[0])

    def rate(self, topic: str) -> float:
        ts = self.index.get(topic, ([], []))[0]
        if len(ts) < 2:
            return 0.0
        span = (ts[-1] - ts[0]) / 1e9
        return (len(ts) - 1) / span if span > 0 else 0.0

    def image_topics(self) -> list:
        return [t for t in self.topics() if self.topic_type.get(t) in IMAGE_TYPES]

    def topics_of_type(self, type_str: str) -> list:
        return [t for t in self.topics() if self.topic_type.get(t) == type_str]

    def stamps(self, topic: str) -> list:
        return self.index.get(topic, ([], []))[0]

    def index_before(self, topic: str, t_ns: int):
        """Index of the newest message of `topic` at or before t_ns (None if none)."""
        ts = self.stamps(topic)
        i = bisect.bisect_right(ts, t_ns) - 1
        return i if i >= 0 else None

    def message_class(self, topic: str):
        type_str = self.topic_type.get(topic)
        if type_str not in self._msg_cls:
            try:
                self._msg_cls[type_str] = get_message(type_str)
            except Exception as exc:
                self._warn(f"no message definition for {type_str} ({exc})")
                self._msg_cls[type_str] = None
        return self._msg_cls[type_str]

    def message_at_index(self, topic: str, i: int):
        """Deserialize message `i` of `topic`. Returns (ts, msg) or (ts, None)."""
        ts_list, loc_list = self.index.get(topic, ([], []))
        if not (0 <= i < len(ts_list)):
            return None
        ts = ts_list[i]
        key = (topic, i)
        if key in self._cache:
            self._cache.move_to_end(key)
            return ts, self._cache[key]
        cls = self.message_class(topic)
        msg = None
        if cls is not None:
            ci, rid = loc_list[i]
            try:
                blob = self.conns[ci].execute(
                    "select data from messages where id=?", (rid,)).fetchone()
                msg = deserialize_message(bytes(blob[0]), cls) if blob else None
            except Exception as exc:
                self._warn(f"{topic} @ {ts}: {exc}")
        self._cache[key] = msg
        if len(self._cache) > 24:
            self._cache.popitem(last=False)
        return ts, msg

    def sample(self, topic: str, t_ns: int):
        """Newest message of `topic` at or before t_ns."""
        i = self.index_before(topic, t_ns)
        return None if i is None else self.message_at_index(topic, i)

    def odom_trail(self, topic: str, max_points: int = 4000):
        """(N,2) array of x/y from an Odometry topic, decoded once and cached."""
        if topic in self._trail_cache:
            return self._trail_cache[topic]
        n = self.count(topic)
        step = max(1, n // max_points)
        pts = []
        for i in range(0, n, step):
            got = self.message_at_index(topic, i)
            if got and got[1] is not None:
                p = got[1].pose.pose.position
                pts.append((p.x, p.y))
        arr = np.asarray(pts, dtype=float) if pts else np.zeros((0, 2))
        self._trail_cache[topic] = arr
        self._cache.clear()  # trail decoding evicted everything useful anyway
        return arr

    def close(self):
        for conn in self.conns:
            try:
                conn.close()
            except Exception:
                pass
        self.conns = []

    # -- text report ------------------------------------------------------
    def info_text(self, cursor_ns=None) -> str:
        s = self.summary
        start = datetime.fromtimestamp(self.t0 / 1e9) if self.t0 else None
        lines = [
            f"BAG      {self.name}",
            f"path     {self.path}",
            f"start    {start.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] if start else '-'}"
            f"    duration  {self.duration:.2f} s",
            f"storage  {s.storage}    compression {s.compression or 'none'}"
            f"    on disk {human_size(s.size)}",
            f"messages {sum(self.count(t) for t in self.topics())}"
            f"    topics {len(self.topics())}",
            "",
            f"{'TOPIC':<26} {'COUNT':>6} {'Hz':>6}  {'AGE':>7}  SENSOR / TYPE",
            "-" * 96,
        ]
        for t in self.topics():
            age = "     - "
            if cursor_ns is not None:
                i = self.index_before(t, cursor_ns)
                if i is None:
                    age = "  (none)"
                else:
                    age = f"{(cursor_ns - self.stamps(t)[i]) / 1e9:6.2f}s"
            lines.append(f"{t:<26} {self.count(t):>6} {self.rate(t):>6.1f}  {age:>7}  "
                         f"{sensor_label(t, self.topic_type.get(t, '?'))}")
        lines.append("")
        lines.append("MESSAGE TYPES")
        for t in self.topics():
            lines.append(f"  {t:<26} {self.topic_type.get(t, '?')}")
        if self.warnings:
            lines.append("")
            lines.append("WARNINGS")
            for w in dict.fromkeys(self.warnings):
                lines.append(f"  ! {w}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Message -> text
# ---------------------------------------------------------------------------

def summarize(msg, indent=0, max_items=6, depth=0) -> str:
    pad = "  " * indent
    if msg is None:
        return pad + "<undecodable>"
    if depth > 6:
        return pad + "..."
    fields = getattr(msg, "get_fields_and_field_types", None)
    if fields is None:
        return pad + _fmt_scalar(msg)
    out = []
    for name in fields():
        val = getattr(msg, name, None)
        if hasattr(val, "get_fields_and_field_types"):
            out.append(f"{pad}{name}:")
            out.append(summarize(val, indent + 1, max_items, depth + 1))
        elif isinstance(val, (list, tuple, np.ndarray, bytes, bytearray)) or \
                hasattr(val, "typecode"):
            out.append(f"{pad}{name}: {_fmt_seq(val, max_items)}")
        else:
            out.append(f"{pad}{name}: {_fmt_scalar(val)}")
    return "\n".join(out)


def _fmt_scalar(v) -> str:
    if isinstance(v, float):
        return f"{v:.4f}"
    if isinstance(v, str) and len(v) > 80:
        return v[:77] + "..."
    return str(v)


def _fmt_seq(v, max_items) -> str:
    try:
        n = len(v)
    except TypeError:
        return str(v)
    if isinstance(v, (bytes, bytearray)):
        return f"<{n} bytes>"
    if n and hasattr(v[0], "get_fields_and_field_types"):
        body = "\n" + "\n".join(
            "    [%d]\n%s" % (i, summarize(v[i], 3)) for i in range(min(n, 3)))
        return f"[{n} items]{body}" + ("\n    ..." if n > 3 else "")
    head = ", ".join(_fmt_scalar(x) for x in list(v)[:max_items])
    return f"[{n}] {head}" + (", ..." if n > max_items else "")


# ---------------------------------------------------------------------------
# Image decoding
# ---------------------------------------------------------------------------

def image_to_rgb(msg, type_str, depth_range=None):
    """Return (rgb uint8 HxWx3, caption) for an Image/CompressedImage message."""
    if type_str == "sensor_msgs/msg/CompressedImage":
        if cv2 is None:
            return None, "cv2 not available — cannot decode CompressedImage"
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return None, "undecodable compressed frame"
        return bgr[:, :, ::-1].copy(), f"{msg.format}  {bgr.shape[1]}x{bgr.shape[0]}"

    h, w, enc = msg.height, msg.width, msg.encoding
    raw = bytes(msg.data)
    dtype, chans = {
        "mono8": (np.uint8, 1), "8UC1": (np.uint8, 1),
        "mono16": (np.uint16, 1), "16UC1": (np.uint16, 1),
        "bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3), "8UC3": (np.uint8, 3),
        "bgra8": (np.uint8, 4), "rgba8": (np.uint8, 4), "8UC4": (np.uint8, 4),
        "32FC1": (np.float32, 1),
    }.get(enc, (None, None))
    if dtype is None:
        return None, f"unsupported encoding '{enc}' ({w}x{h})"

    itemsize = np.dtype(dtype).itemsize
    stride = (msg.step // itemsize) if msg.step else w * chans
    need = stride * h * itemsize
    if len(raw) < need:
        return None, f"short image payload ({len(raw)} < {need} bytes)"
    arr = np.frombuffer(raw[:need], dtype=dtype).reshape(h, stride)[:, :w * chans]
    if msg.is_bigendian and itemsize > 1:
        arr = arr.byteswap()
    arr = arr.reshape(h, w, chans)

    if chans == 1 and dtype in (np.uint16, np.float32):  # depth
        return _colorize_depth(arr[:, :, 0], enc, depth_range)
    if chans == 1:
        gray = arr[:, :, 0]
        return np.repeat(gray[:, :, None], 3, axis=2), f"{enc}  {w}x{h}"
    if chans == 4:
        arr = arr[:, :, :3]
    rgb = arr[:, :, ::-1].copy() if enc.startswith("bgr") else arr.copy()
    return rgb, f"{enc}  {w}x{h}"


def _colorize_depth(depth, enc, depth_range):
    scale = 0.001 if depth.dtype == np.uint16 else 1.0  # 16UC1 is millimetres
    d = depth.astype(np.float32) * scale
    valid = np.isfinite(d) & (d > 0)
    if not valid.any():
        return np.zeros(d.shape + (3,), np.uint8), f"{enc}  {d.shape[1]}x{d.shape[0]}  (all invalid)"
    if depth_range is None:
        lo, hi = np.percentile(d[valid], (2, 98))
    else:
        lo, hi = depth_range
    hi = max(hi, lo + 1e-3)
    norm = np.clip((d - lo) / (hi - lo), 0, 1)
    u8 = (norm * 255).astype(np.uint8)
    if cv2 is not None:
        rgb = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)[:, :, ::-1].copy()
    else:
        rgb = np.repeat(u8[:, :, None], 3, axis=2)
    rgb[~valid] = 0
    dv = d[valid]
    cap = (f"{enc}  {d.shape[1]}x{d.shape[0]}   range {lo:.2f}-{hi:.2f} m   "
           f"min {dv.min():.2f}  med {np.median(dv):.2f}  max {dv.max():.2f} m   "
           f"valid {100.0 * valid.mean():.0f}%")
    return rgb, cap


# ---------------------------------------------------------------------------
# Text-mode entry points
# ---------------------------------------------------------------------------

def cmd_list(roots):
    bags = find_bags(roots)
    if not bags:
        print("No rosbag2 directories found under: " + ", ".join(roots))
        return 1
    print(f"{'BAG':<34} {'STARTED':<19}  {'DUR':>8}  {'MESSAGES':>12}  {'SIZE':>9}  COMPRESSION")
    print("-" * 110)
    empty = 0
    for b in bags:
        s = BagSummary(b)
        if s.is_empty:
            empty += 1
        print(s.list_line())
    print("-" * 110)
    print(f"{len(bags)} bags ({empty} with zero messages)")
    return 0


def cmd_info(paths):
    rc = 0
    for p in paths:
        p = os.path.abspath(os.path.expanduser(p))
        print("=" * 96)
        try:
            bag = BagIndex(p)
        except Exception as exc:
            print(f"{p}\n  ERROR: {exc}")
            rc = 1
            continue
        print(bag.info_text())
        bag.close()
    return rc


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

def run_gui(roots, sync_topic=None, _hook=None):
    from PyQt5 import QtCore, QtGui, QtWidgets

    Qt = QtCore.Qt
    MONO = QtGui.QFont("monospace", 9)
    MONO.setStyleHint(QtGui.QFont.TypeWriter)

    class PlotPanel(QtWidgets.QWidget):
        """Top-down laser scan (left) and odometry trail (right)."""

        def __init__(self):
            super().__init__()
            self.scan = None
            self.trail = np.zeros((0, 2))
            self.pose = None
            self.show_trail = True
            self.setMinimumHeight(240)
            self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                               QtWidgets.QSizePolicy.Expanding)

        def set_data(self, scan, trail, pose):
            self.scan, self.trail, self.pose = scan, trail, pose
            self.update()

        def paintEvent(self, _):
            p = QtGui.QPainter(self)
            p.setRenderHint(QtGui.QPainter.Antialiasing)
            p.fillRect(self.rect(), QtGui.QColor(20, 22, 26))
            w, h = self.width(), self.height()
            half = w // 2
            self._draw_scan(p, QtCore.QRect(0, 0, half, h))
            self._draw_trail(p, QtCore.QRect(half, 0, w - half, h))
            p.setPen(QtGui.QColor(60, 64, 72))
            p.drawLine(half, 0, half, h)

        def _title(self, p, rect, text, colour=QtGui.QColor(150, 158, 170)):
            p.save()
            p.setClipRect(rect)
            p.setPen(colour)
            metrics = QtGui.QFontMetrics(p.font())
            p.drawText(rect.left() + 8, rect.top() + 16,
                       metrics.elidedText(text, Qt.ElideRight, rect.width() - 16))
            p.restore()

        def _draw_scan(self, p, rect):
            if self.scan is None:
                self._title(p, rect, "LaserScan — no /scan message at this time")
                return
            msg = self.scan
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            n = len(ranges)
            angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
            ok = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            r, a = ranges[ok], angles[ok]
            rmax = max(float(r.max()) if r.size else 1.0, 1.0)
            cx, cy = rect.center().x(), rect.center().y()
            scale = (min(rect.width(), rect.height()) * 0.45) / rmax

            p.setPen(QtGui.QPen(QtGui.QColor(45, 50, 58), 1))
            for ring in range(1, int(rmax) + 1):
                rad = ring * scale
                p.drawEllipse(QtCore.QPointF(cx, cy), rad, rad)
            # ROS base frame: +x forward (up on screen), +y left (left on screen)
            x = cx - r * np.sin(a) * scale
            y = cy - r * np.cos(a) * scale
            p.setPen(QtGui.QPen(QtGui.QColor(120, 230, 180), 2))
            for xi, yi in zip(x, y):
                p.drawPoint(QtCore.QPointF(float(xi), float(yi)))
            p.setPen(QtGui.QPen(QtGui.QColor(240, 200, 90), 2))
            p.drawLine(int(cx), int(cy), int(cx), int(cy - 12))
            p.drawEllipse(QtCore.QPointF(cx, cy), 3, 3)
            self._title(p, rect, f"LaserScan  {int(ok.sum())}/{n} pts   "
                                 f"max {rmax:.1f} m   rings = 1 m")

        def _draw_trail(self, p, rect):
            if not self.show_trail or self.trail is None or len(self.trail) == 0:
                self._title(p, rect, "Odometry trail — hidden (o)" if not self.show_trail
                            else "Odometry trail — no /odom in this bag")
                return
            t = self.trail
            x0, y0 = t[:, 0].min(), t[:, 1].min()
            x1, y1 = t[:, 0].max(), t[:, 1].max()
            span = max(x1 - x0, y1 - y0, 0.5)
            m = 28
            scale = (min(rect.width(), rect.height()) - 2 * m) / span
            ox = rect.left() + m + (rect.width() - 2 * m - (x1 - x0) * scale) / 2
            oy = rect.bottom() - m - (rect.height() - 2 * m - (y1 - y0) * scale) / 2

            def to_px(px, py):
                return QtCore.QPointF(ox + (px - x0) * scale, oy - (py - y0) * scale)

            path = QtGui.QPainterPath(to_px(*t[0]))
            for px, py in t[1:]:
                path.lineTo(to_px(px, py))
            p.setPen(QtGui.QPen(QtGui.QColor(90, 150, 240), 2))
            p.drawPath(path)
            if self.pose is not None:
                pt = to_px(self.pose[0], self.pose[1])
                p.setPen(QtGui.QPen(QtGui.QColor(240, 120, 120), 2))
                p.setBrush(QtGui.QColor(240, 120, 120))
                p.drawEllipse(pt, 4, 4)
                p.setBrush(Qt.NoBrush)
                yaw = self.pose[2]
                p.drawLine(pt, QtCore.QPointF(pt.x() + 14 * np.cos(yaw),
                                              pt.y() - 14 * np.sin(yaw)))
            self._title(p, rect, f"Odometry trail  {span:.1f} m across  "
                                 f"({len(t)} pts, odom frame)")

    class Viewer(QtWidgets.QMainWindow):
        STEP_SMALL_NS = 1_000_000_000
        STEP_BIG_NS = 10_000_000_000

        def __init__(self):
            super().__init__()
            self.setWindowTitle("X3 rosbag viewer")
            self.resize(1500, 900)
            self.roots = roots
            self.bag = None
            self.bags = []
            self.cursor = 0
            self.speed = 1.0
            self.playing = False
            self.image_topic = None
            self.sync_topic = sync_topic
            self.fixed_depth = False
            self._last_tick = None
            self._build_ui()
            self._install_keys()
            self.rescan()

        # -- layout -------------------------------------------------------
        def _build_ui(self):
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            root = QtWidgets.QVBoxLayout(central)
            root.setContentsMargins(6, 6, 6, 4)

            split = QtWidgets.QSplitter(Qt.Horizontal)
            root.addWidget(split, 1)

            # left — bag list
            left = QtWidgets.QWidget()
            lv = QtWidgets.QVBoxLayout(left)
            lv.setContentsMargins(0, 0, 0, 0)
            lv.addWidget(QtWidgets.QLabel("Bags  (n / p to change)"))
            self.bag_list = QtWidgets.QListWidget()
            self.bag_list.setFocusPolicy(Qt.NoFocus)
            small = QtGui.QFont(self.bag_list.font())
            small.setPointSize(max(7, small.pointSize() - 1))
            self.bag_list.setFont(small)
            self.bag_list.currentRowChanged.connect(self._on_bag_row)
            lv.addWidget(self.bag_list, 1)
            left.setMinimumWidth(270)
            split.addWidget(left)

            # centre — image + plots
            centre = QtWidgets.QSplitter(Qt.Vertical)
            img_box = QtWidgets.QWidget()
            iv = QtWidgets.QVBoxLayout(img_box)
            iv.setContentsMargins(0, 0, 0, 0)
            self.image_label = QtWidgets.QLabel("no image topic")
            self.image_label.setAlignment(Qt.AlignCenter)
            self.image_label.setMinimumHeight(260)
            self.image_label.setStyleSheet("background:#14161a;color:#8a92a0")
            self.image_caption = QtWidgets.QLabel("")
            self.image_caption.setFont(MONO)
            iv.addWidget(self.image_label, 1)
            iv.addWidget(self.image_caption)
            centre.addWidget(img_box)
            self.plot = PlotPanel()
            centre.addWidget(self.plot)
            centre.setSizes([520, 340])
            split.addWidget(centre)

            # right — text
            right = QtWidgets.QTabWidget()
            right.setFocusPolicy(Qt.NoFocus)
            self.info_view = QtWidgets.QPlainTextEdit()
            self.msg_view = QtWidgets.QPlainTextEdit()
            for v in (self.info_view, self.msg_view):
                v.setReadOnly(True)
                v.setFont(MONO)
                v.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
                v.setFocusPolicy(Qt.NoFocus)
            right.addTab(self.info_view, "Topics / sensors")
            right.addTab(self.msg_view, "Messages at cursor")
            right.setMinimumWidth(460)
            split.addWidget(right)
            split.setSizes([300, 720, 480])
            split.setStretchFactor(1, 1)

            # bottom — transport
            bar = QtWidgets.QHBoxLayout()
            self.play_btn = QtWidgets.QPushButton("▶")
            self.play_btn.setFixedWidth(38)
            self.play_btn.setFocusPolicy(Qt.NoFocus)
            self.play_btn.clicked.connect(self.toggle_play)
            bar.addWidget(self.play_btn)
            self.slider = QtWidgets.QSlider(Qt.Horizontal)
            self.slider.setRange(0, 1000)
            self.slider.setFocusPolicy(Qt.NoFocus)
            self.slider.sliderMoved.connect(self._on_slider)
            self.slider.valueChanged.connect(self._on_slider)
            bar.addWidget(self.slider, 1)
            self.time_label = QtWidgets.QLabel("--")
            self.time_label.setFont(MONO)
            self.time_label.setMinimumWidth(340)
            bar.addWidget(self.time_label)
            root.addLayout(bar)

            self.status = self.statusBar()
            hint = QtWidgets.QLabel("Space play · ←/→ frame · Shift ±1s · Ctrl ±10s · "
                                    "n/p bag · i image · s sync · ? help")
            hint.setStyleSheet("color:#7d8594")
            self.status.addPermanentWidget(hint)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self._tick)
            self.timer.start(33)

        def _install_keys(self):
            binds = [
                ("Space", self.toggle_play),
                ("Right", lambda: self.step_message(+1)),
                ("Left", lambda: self.step_message(-1)),
                ("Shift+Right", lambda: self.step_time(+self.STEP_SMALL_NS)),
                ("Shift+Left", lambda: self.step_time(-self.STEP_SMALL_NS)),
                ("Ctrl+Right", lambda: self.step_time(+self.STEP_BIG_NS)),
                ("Ctrl+Left", lambda: self.step_time(-self.STEP_BIG_NS)),
                ("Home", lambda: self.seek_to(self.bag.t0 if self.bag else 0)),
                ("End", lambda: self.seek_to(self.bag.t1 if self.bag else 0)),
                ("n", lambda: self.change_bag(+1)),
                ("p", lambda: self.change_bag(-1)),
                ("PgDown", lambda: self.change_bag(+1)),
                ("PgUp", lambda: self.change_bag(-1)),
                ("i", self.cycle_image_topic),
                ("s", self.cycle_sync_topic),
                ("+", lambda: self.change_speed(2.0)),
                ("=", lambda: self.change_speed(2.0)),
                ("-", lambda: self.change_speed(0.5)),
                ("o", self.toggle_trail),
                ("d", self.toggle_depth_range),
                ("r", self.rescan),
                ("?", self.show_help),
                ("h", self.show_help),
                ("q", self.close),
                ("Esc", self.close),
            ]
            for seq, fn in binds:
                sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self)
                sc.setContext(Qt.ApplicationShortcut)
                sc.activated.connect(fn)

        # -- bag management -----------------------------------------------
        def rescan(self):
            keep = self.bag.path if self.bag else None
            self.bags = find_bags(self.roots)
            # Two dirs can share a basename (e.g. bags/x and bags/domain_adapt/x),
            # so disambiguate those with their parent directory.
            names = [os.path.basename(b.rstrip("/")) for b in self.bags]
            dupes = {n for n in names if names.count(n) > 1}

            self.bag_list.blockSignals(True)
            self.bag_list.clear()
            first_useful = None
            for i, b in enumerate(self.bags):
                s = BagSummary(b)
                shown = names[i]
                if shown in dupes:
                    shown = os.path.join(os.path.basename(os.path.dirname(b)), shown)
                item = QtWidgets.QListWidgetItem(s.label(shown))
                if s.is_empty:
                    item.setForeground(QtGui.QColor(130, 130, 130))
                elif first_useful is None:
                    first_useful = i
                item.setData(Qt.UserRole, b)
                item.setToolTip(b)
                self.bag_list.addItem(item)
            self.bag_list.blockSignals(False)
            if not self.bags:
                self.status.showMessage("No bags found under: " + ", ".join(self.roots))
                return
            # Open on a bag that actually has data rather than the first empty one.
            row = self.bags.index(keep) if keep in self.bags else (first_useful or 0)
            self.bag_list.setCurrentRow(row)
            self._load_bag(row)

        def _on_bag_row(self, row):
            if 0 <= row < len(self.bags):
                self._load_bag(row)

        def _load_bag(self, row):
            self.playing = False
            self.play_btn.setText("▶")
            if self.bag:
                self.bag.close()
                self.bag = None
            path = self.bags[row]
            try:
                self.bag = BagIndex(path)
            except Exception as exc:
                self.info_view.setPlainText(f"{path}\n\nCould not open this bag:\n  {exc}")
                self.msg_view.clear()
                self.image_label.setText("—")
                self.image_caption.setText("")
                self.plot.set_data(None, None, None)
                self.status.showMessage(f"{os.path.basename(path)}: {exc}")
                return
            imgs = self.bag.image_topics()
            self.image_topic = imgs[0] if imgs else None
            if self.sync_topic not in self.bag.topics():
                self.sync_topic = self._default_sync_topic()
            # Start where the sync topic actually has data, so the first view is
            # not blank on bags that open with a burst of /tf before the sensors.
            first = self.bag.stamps(self.sync_topic) if self.sync_topic else []
            self.cursor = first[0] if first else self.bag.t0
            n = sum(self.bag.count(t) for t in self.bag.topics())
            msg = f"{self.bag.name}   {n} messages   {self.bag.duration:.1f} s"
            if self.bag.warnings:
                msg += f"   ({len(set(self.bag.warnings))} warning(s) — see Topics tab)"
            self.status.showMessage(msg)
            self.setWindowTitle(f"X3 rosbag viewer — {self.bag.name}")
            self.refresh(full=True)

        def _default_sync_topic(self):
            if not self.bag or not self.bag.topics():
                return None
            imgs = self.bag.image_topics()
            if imgs:
                return imgs[0]
            for pref in ("/scan", "/odom"):
                if pref in self.bag.topics():
                    return pref
            return max(self.bag.topics(), key=self.bag.count)

        def change_bag(self, delta):
            if not self.bags:
                return
            row = (self.bag_list.currentRow() + delta) % len(self.bags)
            self.bag_list.setCurrentRow(row)

        # -- transport ----------------------------------------------------
        def toggle_play(self):
            if not self.bag:
                return
            self.playing = not self.playing
            self._last_tick = time.monotonic()
            self.play_btn.setText("❚❚" if self.playing else "▶")

        def change_speed(self, factor):
            self.speed = min(16.0, max(0.125, self.speed * factor))
            self.refresh()

        def seek_to(self, t_ns):
            if not self.bag:
                return
            self.cursor = int(min(max(t_ns, self.bag.t0), self.bag.t1))
            self.refresh()

        def step_time(self, delta_ns):
            self.playing = False
            self.play_btn.setText("▶")
            self.seek_to(self.cursor + delta_ns)

        def step_message(self, direction):
            """Move to the next/previous message of the sync topic."""
            if not self.bag:
                return
            self.playing = False
            self.play_btn.setText("▶")
            topic = self.sync_topic
            stamps = self.bag.stamps(topic) if topic else []
            if not stamps:
                self.step_time(direction * 100_000_000)
                return
            i = self.bag.index_before(topic, self.cursor)
            if direction > 0:
                j = 0 if i is None else min(i + 1, len(stamps) - 1)
            else:
                j = 0 if i is None else max(i - 1, 0)
                if stamps[i] < self.cursor:  # cursor drifted past it — go back to it
                    j = i
            self.seek_to(stamps[j])

        def _on_slider(self, value):
            if not self.bag or self.bag.t1 == self.bag.t0:
                return
            t = self.bag.t0 + (self.bag.t1 - self.bag.t0) * value / 1000.0
            if abs(t - self.cursor) > (self.bag.t1 - self.bag.t0) / 2000.0:
                self.playing = False
                self.play_btn.setText("▶")
                self.cursor = int(t)
                self.refresh()

        def _tick(self):
            if not (self.playing and self.bag):
                return
            now = time.monotonic()
            dt = now - (self._last_tick or now)
            self._last_tick = now
            self.cursor += int(dt * self.speed * 1e9)
            if self.cursor >= self.bag.t1:
                self.cursor = self.bag.t1
                self.playing = False
                self.play_btn.setText("▶")
            self.refresh()

        # -- display toggles ----------------------------------------------
        def cycle_image_topic(self):
            if not self.bag:
                return
            imgs = self.bag.image_topics()
            if not imgs:
                return
            i = imgs.index(self.image_topic) if self.image_topic in imgs else -1
            self.image_topic = imgs[(i + 1) % len(imgs)]
            self.refresh()

        def cycle_sync_topic(self):
            if not self.bag:
                return
            ts = [t for t in self.bag.topics() if self.bag.count(t) > 0]
            if not ts:
                return
            i = ts.index(self.sync_topic) if self.sync_topic in ts else -1
            self.sync_topic = ts[(i + 1) % len(ts)]
            self.refresh()

        def toggle_trail(self):
            self.plot.show_trail = not self.plot.show_trail
            self.refresh()

        def toggle_depth_range(self):
            self.fixed_depth = not self.fixed_depth
            self.refresh()

        def show_help(self):
            QtWidgets.QMessageBox.information(self, "Keys", __doc__.split("Keys (GUI)")[1]
                                              .split("Notes on these bags")[0].strip("-\n "))

        # -- rendering ----------------------------------------------------
        def refresh(self, full=False):
            if not self.bag:
                return
            self._render_image()
            self._render_plots()
            self._render_text(full)
            self._render_transport()

        def _render_image(self):
            topic = self.image_topic
            if not topic:
                self.image_label.setText("no image topic in this bag")
                self.image_caption.setText("")
                return
            got = self.bag.sample(topic, self.cursor)
            if got is None or got[1] is None:
                self.image_label.setText(f"{topic}: no frame at this time")
                self.image_caption.setText(f"{topic}")
                return
            ts, msg = got
            rng = (0.3, 6.0) if self.fixed_depth else None
            rgb, cap = image_to_rgb(msg, self.bag.topic_type[topic], rng)
            age = (self.cursor - ts) / 1e9
            self.image_caption.setText(
                f"{topic}   {cap}   frame_id={getattr(msg.header, 'frame_id', '?')}   "
                f"age {age:.2f}s   [i] cycle image   [d] "
                f"{'fixed 0.3-6 m' if self.fixed_depth else 'auto range'}")
            if rgb is None:
                self.image_label.setText(cap)
                return
            rgb = np.ascontiguousarray(rgb)
            h, w, _ = rgb.shape
            qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(qimg.copy())
            self.image_label.setPixmap(pix.scaled(self.image_label.size(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation))

        def _render_plots(self):
            scan = None
            for t in self.bag.topics_of_type("sensor_msgs/msg/LaserScan"):
                got = self.bag.sample(t, self.cursor)
                if got and got[1] is not None:
                    scan = got[1]
                    break
            trail, pose = np.zeros((0, 2)), None
            odoms = self.bag.topics_of_type("nav_msgs/msg/Odometry")
            if odoms and self.plot.show_trail:
                trail = self.bag.odom_trail(odoms[0])
                got = self.bag.sample(odoms[0], self.cursor)
                if got and got[1] is not None:
                    p = got[1].pose.pose.position
                    q = got[1].pose.pose.orientation
                    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y ** 2 + q.z ** 2))
                    pose = (p.x, p.y, yaw)
            self.plot.set_data(scan, trail, pose)

        def _render_text(self, full):
            if full:
                self.info_view.setPlainText(self.bag.info_text(self.cursor))
            else:
                bar = self.info_view.verticalScrollBar().value()
                self.info_view.setPlainText(self.bag.info_text(self.cursor))
                self.info_view.verticalScrollBar().setValue(bar)

            bar = self.msg_view.verticalScrollBar().value()
            out = []
            for t in self.bag.topics():
                got = self.bag.sample(t, self.cursor)
                head = f"── {t}  [{self.bag.topic_type.get(t, '?')}]"
                if got is None:
                    out.append(f"{head}\n   (nothing recorded before the cursor yet)\n")
                    continue
                ts, msg = got
                out.append(f"{head}   t-{(self.cursor - ts) / 1e9:.3f}s\n"
                           f"{summarize(msg, 1)}\n")
            self.msg_view.setPlainText("\n".join(out))
            self.msg_view.verticalScrollBar().setValue(bar)

        def _render_transport(self):
            span = self.bag.t1 - self.bag.t0
            elapsed = (self.cursor - self.bag.t0) / 1e9
            if span > 0:
                v = int(1000 * (self.cursor - self.bag.t0) / span)
                self.slider.blockSignals(True)
                self.slider.setValue(max(0, min(1000, v)))
                self.slider.blockSignals(False)
            wall = datetime.fromtimestamp(self.cursor / 1e9).strftime("%H:%M:%S.%f")[:-3]
            sync = self.sync_topic or "-"
            i = self.bag.index_before(sync, self.cursor) if self.sync_topic else None
            frame = "-" if i is None else f"{i + 1}/{self.bag.count(sync)}"
            self.time_label.setText(
                f"t={elapsed:7.2f}/{self.bag.duration:.2f}s  {wall}  "
                f"x{self.speed:g}  sync[s]={sync} {frame}")

        def resizeEvent(self, ev):
            super().resizeEvent(ev)
            if self.bag:
                self._render_image()

        def closeEvent(self, ev):
            if self.bag:
                self.bag.close()
            super().closeEvent(ev)

    app = QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    win = Viewer()
    win.show()
    if _hook is not None:          # used by the offscreen smoke test
        return _hook(app, win)
    return app.exec_()


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Browse and scrub rosbag2 recordings from the X3 robot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Keys (GUI)")[1].split("Notes on these bags")[0])
    ap.add_argument("paths", nargs="*", help="bag dirs or dirs containing bags")
    ap.add_argument("--list", action="store_true", help="print a table of bags and exit")
    ap.add_argument("--info", action="store_true",
                    help="print topics/sensors of the given bag(s) and exit")
    ap.add_argument("--sync-topic", default=None,
                    help="topic the left/right keys step through")
    args = ap.parse_args(argv)

    roots = args.paths or DEFAULT_ROOTS
    if args.info:
        targets = args.paths or find_bags(DEFAULT_ROOTS)
        if not targets:
            print("No bags found.")
            return 1
        return cmd_info(targets)
    if args.list:
        return cmd_list(roots)
    return run_gui(roots, args.sync_topic)


if __name__ == "__main__":
    sys.exit(main())
