#!/usr/bin/env python3
"""Extract small offline fixtures from a recorded rosbag.

The bag corpus (~8.8 GB in ~/EE_244_Final_Project/bags) is far too large to live in
git, but without *some* recorded data you cannot touch perception or odometry code
unless the robot is powered on and reachable. This carves a short window out of one
real bag into a few megabytes of plain numpy, which `src/replay_bridge.py` replays as
a drop-in for `ROS2Bridge`.

    python3 scripts/make_fixtures.py                       # defaults below
    python3 scripts/make_fixtures.py --bag <name-or-path> --seconds 30

Fixtures are stored **decoded**, not as serialized ROS messages, so replaying them
needs numpy and nothing else -- no rclpy, no ROS install, no DDS. That is the whole
point: the fixture has to work on a machine that has never had ROS on it.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from bag_viewer import BagIndex, find_bags  # noqa: E402  (needs sys.path first)

# Chosen because its topic set is exactly what ROS2Bridge subscribes to
# (/scan, /odom, /camera/depth/image_raw, /tf) -- the 20260731 bags carry /oak/*
# instead, which the bridge does not read. 11546 msgs over 112 s.
DEFAULT_BAG = "domain_adapt_20260529_192203"
DEFAULT_ROOTS = [
    os.path.expanduser("~/EE_244_Final_Project/bags"),
    os.path.expanduser("~/bags"),
    "./bags",
]

SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
DEPTH_TOPICS = ("/camera/depth/image_raw", "/camera/depth_image", "/oak/depth/image_raw")


def resolve_bag(name: str, roots: list[str]) -> str:
    """Accept a full path or a bare bag name."""
    if os.path.isdir(name):
        return name
    matches = [b for b in find_bags(roots) if os.path.basename(b.rstrip("/")) == name]
    if not matches:
        raise SystemExit(
            f"bag {name!r} not found under {roots}.\n"
            f"List what's available with: python3 src/bag_viewer.py --list"
        )
    return matches[0]


def quaternion_to_yaw(q) -> float:
    """Yaw only -- the robot is planar, so roll/pitch carry no information here."""
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def indices_in_window(index: BagIndex, topic: str, t0: int, t1: int) -> list[int]:
    stamps = index.stamps(topic)
    return [i for i, ts in enumerate(stamps) if t0 <= ts <= t1]


def extract_scan(index: BagIndex, t0: int, t1: int) -> dict | None:
    idxs = indices_in_window(index, SCAN_TOPIC, t0, t1)
    if not idxs:
        return None
    ranges, stamps, angle_min, angle_inc, first = [], [], [], [], None
    for i in idxs:
        got = index.message_at_index(SCAN_TOPIC, i)
        if not got or got[1] is None:
            continue
        ts, msg = got
        if first is None:
            first = msg
        ranges.append(np.asarray(msg.ranges, dtype=np.float32))
        angle_min.append(msg.angle_min)
        angle_inc.append(msg.angle_increment)
        stamps.append(ts)
    if not ranges:
        return None
    # The YDLidar X3 does NOT emit a fixed beam count -- it varies scan to scan
    # (observed 2782..2794 on this bag) because the driver bins whatever samples
    # arrived in one revolution. So the array is padded to the widest scan with
    # NaN, and `counts` carries the true length. Angle geometry is per-scan for
    # the same reason: angle_increment is derived from the actual sample count.
    width = max(len(r) for r in ranges)
    padded = np.full((len(ranges), width), np.nan, dtype=np.float32)
    for row, r in enumerate(ranges):
        padded[row, :len(r)] = r
    return {
        "ranges": padded,
        "counts": np.asarray([len(r) for r in ranges], dtype=np.int32),
        "angle_min": np.asarray(angle_min, dtype=np.float32),
        "angle_increment": np.asarray(angle_inc, dtype=np.float32),
        "stamps_ns": np.asarray(stamps, dtype=np.int64),
        "range_min": float(first.range_min),
        "range_max": float(first.range_max),
        "frame_id": first.header.frame_id,
    }


def extract_odom(index: BagIndex, t0: int, t1: int) -> dict | None:
    idxs = indices_in_window(index, ODOM_TOPIC, t0, t1)
    if not idxs:
        return None
    pose, twist, stamps = [], [], []
    for i in idxs:
        got = index.message_at_index(ODOM_TOPIC, i)
        if not got or got[1] is None:
            continue
        ts, msg = got
        p = msg.pose.pose.position
        t = msg.twist.twist
        pose.append((p.x, p.y, quaternion_to_yaw(msg.pose.pose.orientation)))
        twist.append((t.linear.x, t.linear.y, t.angular.z))
        stamps.append(ts)
    if not pose:
        return None
    return {
        "pose_xytheta": np.asarray(pose, dtype=np.float64),
        "twist_vxvywz": np.asarray(twist, dtype=np.float64),
        "stamps_ns": np.asarray(stamps, dtype=np.int64),
    }


def extract_depth(index: BagIndex, t0: int, t1: int, max_frames: int) -> dict | None:
    topic = next((t for t in DEPTH_TOPICS if index.count(t)), None)
    if topic is None:
        return None
    idxs = indices_in_window(index, topic, t0, t1)
    if not idxs:
        return None
    # Depth dominates fixture size (640x480x2 B/frame), so subsample hard --
    # replay only needs enough frames to exercise the decode path, not full rate.
    step = max(1, len(idxs) // max_frames)
    picked = idxs[::step][:max_frames]
    frames, stamps, encoding, first = [], [], None, None
    for i in picked:
        got = index.message_at_index(topic, i)
        if not got or got[1] is None:
            continue
        ts, msg = got
        if first is None:
            first, encoding = msg, msg.encoding
        if msg.encoding != encoding:
            continue
        if msg.encoding != "16UC1":
            raise SystemExit(
                f"{topic} encoding is {msg.encoding!r}; this extractor only handles 16UC1 "
                "(millimetre depth). Extend it if you need another encoding."
            )
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        frames.append(arr.reshape(msg.height, msg.width))
        stamps.append(ts)
    if not frames:
        return None
    return {
        "frames_mm": np.stack(frames),
        "stamps_ns": np.asarray(stamps, dtype=np.int64),
        "encoding": encoding,
        "topic": topic,
        "frame_id": first.header.frame_id,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", default=DEFAULT_BAG, help=f"bag name or path (default: {DEFAULT_BAG})")
    parser.add_argument("--seconds", type=float, default=30.0, help="window length (default: 30)")
    parser.add_argument("--skip", type=float, default=5.0,
                        help="seconds to skip at the start, past spin-up (default: 5)")
    parser.add_argument("--depth-frames", type=int, default=12,
                        help="max depth frames to keep (default: 12)")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "fixtures",
                        help="output directory (default: fixtures/)")
    args = parser.parse_args()

    bag_path = resolve_bag(args.bag, DEFAULT_ROOTS)
    index = BagIndex(bag_path, warn=lambda m: print(f"  warn: {m}", file=sys.stderr))
    try:
        if not index.t0:
            raise SystemExit(f"{bag_path} has no messages")
        t0 = index.t0 + int(args.skip * 1e9)
        t1 = t0 + int(args.seconds * 1e9)
        print(f"bag      {bag_path}")
        print(f"window   +{args.skip:.0f}s .. +{args.skip + args.seconds:.0f}s of {index.duration:.1f}s")

        scan = extract_scan(index, t0, t1)
        odom = extract_odom(index, t0, t1)
        depth = extract_depth(index, t0, t1, args.depth_frames)
    finally:
        index.close()

    if scan is None and odom is None and depth is None:
        raise SystemExit("window contained no usable messages -- try a different --skip")

    args.out.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source_bag": os.path.basename(bag_path.rstrip("/")),
        "source_path": bag_path,
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "generator": "scripts/make_fixtures.py",
        "window_seconds": args.seconds,
        "skip_seconds": args.skip,
        "topics": {},
    }

    for name, data in (("scan", scan), ("odom", odom), ("depth", depth)):
        if data is None:
            print(f"  {name:<6} -- not present in this bag, skipped")
            continue
        arrays = {k: v for k, v in data.items() if isinstance(v, np.ndarray)}
        scalars = {k: v for k, v in data.items() if not isinstance(v, np.ndarray)}
        path = args.out / f"{name}.npz"
        np.savez_compressed(path, **arrays, **{f"_meta_{k}": v for k, v in scalars.items()})
        count = len(next(iter(arrays.values())))
        manifest["topics"][name] = dict(scalars, count=count,
                                        file=f"{name}.npz",
                                        bytes=path.stat().st_size)
        print(f"  {name:<6} {count:5d} samples  {path.stat().st_size / 1e6:6.2f} MB  -> {path.name}")

    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    total = sum(p.stat().st_size for p in args.out.glob("*"))
    print(f"total    {total / 1e6:.2f} MB in {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
