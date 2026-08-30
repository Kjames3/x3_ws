#!/usr/bin/env python3
"""save_slam_map.py — save a slam_toolbox map safely, and PROVE it is the real one.

    python3 scripts/save_slam_map.py <name>

Why this exists
---------------
A whole mapping run was lost because AMCL's map_server was left running. Both
map_server and slam_toolbox publish /map, so /slam_toolbox/save_map captured the
wrong source. The result looked plausible — same dimensions, same origin, a
sensible-looking file — and was only caught by comparing it cell-by-cell against
the previous map.

The tell is simple and worth automating: a genuine SLAM map has a large UNKNOWN
region outside the walls. A map republished by map_server, or any map that has
been through a round trip, can come back with unknown rewritten as free. Zero
unknown cells means you saved the wrong thing.

This script therefore:
  1. refuses to run while map_server is alive,
  2. requires slam_toolbox to be running,
  3. saves both the occupancy grid AND the pose graph (the latter is what lets a
     future session EXTEND the map instead of remapping from scratch — easy to
     forget, impossible to recreate once slam_toolbox exits),
  4. verifies the saved PGM really looks like a fresh SLAM map.
"""

import os
import subprocess
import sys

MAPS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "src", "yahboomcar_nav", "maps")
# Trinary PGM encoding used by nav2's map_saver.
OCCUPIED, UNKNOWN, FREE = 0, 205, 254


def procs(pattern):
    try:
        out = subprocess.run(["pgrep", "-fc", pattern], capture_output=True, text=True)
        return int(out.stdout.strip() or 0)
    except Exception:
        return 0


def _ros(cmd, timeout=180):
    full = (f'source /opt/ros/humble/setup.bash && '
            f'source {os.path.expanduser("~")}/x3_ws/install/setup.bash && '
            f'export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-42}} && {cmd}')
    r = subprocess.run(["bash", "-c", full], capture_output=True, text=True,
                       timeout=timeout)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def call_service(srv, srv_type, payload):
    rc, out = _ros(f'ros2 service call {srv} {srv_type} "{payload}"', timeout=120)
    return rc == 0 and "result=0" in out, out


def save_grid(base):
    """Save the occupancy grid with nav2's map_saver_cli.

    NOT slam_toolbox's /slam_toolbox/save_map service: that spins up an internal
    map_saver with a VOLATILE subscription and a ~2 s window, so it only catches
    a map if slam_toolbox happens to publish during those two seconds. In
    practice it fails with "Failed to spin map subscription" and returns 255.
    slam_toolbox latches /map, so a transient_local subscription gets the
    retained message immediately and deterministically.
    """
    out_dir, name = os.path.dirname(base), os.path.basename(base)
    rc, out = _ros(
        f'cd "{out_dir}" && ros2 run nav2_map_server map_saver_cli -f "{name}" '
        f'--ros-args -p map_subscribe_transient_local:=true '
        f'-p save_map_timeout:=20.0')
    return ("Map saved successfully" in out), out


def map_publishers():
    """Who is publishing /map, by node name.

    A process check is not enough. A map_server killed with SIGKILL — or one on
    another machine on the same ROS_DOMAIN_ID — leaves a live publisher on the
    topic that `pgrep` on this host cannot see. Since the save reads the /map
    TOPIC, any second publisher can win the race.
    """
    rc, out = _ros("ros2 topic info /map --verbose", timeout=60)
    nodes, current = [], None
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("Node name:"):
            current = line.split(":", 1)[1].strip()
        elif line.startswith("Endpoint type: PUBLISHER") and current:
            nodes.append(current)
    return nodes


def verify(pgm_path):
    """A real SLAM map is trinary and has substantial unknown space."""
    try:
        import numpy as np
        import cv2
    except ImportError:
        print("  (numpy/cv2 unavailable — skipping verification)")
        return True
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"  FAIL: cannot read {pgm_path}")
        return False
    vals, counts = np.unique(img, return_counts=True)
    hist = dict(zip(vals.tolist(), counts.tolist()))
    total = int(img.size)
    unknown = hist.get(UNKNOWN, 0)
    occupied = hist.get(OCCUPIED, 0)
    print(f"  size={img.shape[1]}x{img.shape[0]}  values={hist}")
    ok = True
    if unknown == 0:
        print("  FAIL: ZERO unknown cells. A genuine SLAM map has unmapped space "
              "outside the walls — you almost certainly saved map_server's copy.")
        ok = False
    elif unknown < 0.05 * total:
        print(f"  WARN: only {100*unknown/total:.1f}% unknown — unusually low.")
    if occupied == 0:
        print("  FAIL: no occupied cells at all.")
        ok = False
    if ok:
        print(f"  OK: {100*occupied/total:.1f}% occupied, "
              f"{100*unknown/total:.1f}% unknown — looks like a real SLAM map.")
    return ok


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    name = sys.argv[1]
    if os.sep in name:
        print("give a bare map name, not a path")
        sys.exit(2)

    n_map_server = procs("[m]ap_server")
    n_slam = procs("[a]sync_slam_toolbox")
    print(f"map_server processes: {n_map_server}   slam_toolbox processes: {n_slam}")

    if n_map_server:
        print("\nREFUSING TO SAVE: map_server is running.\n"
              "It publishes /map alongside slam_toolbox, and the save would capture\n"
              "the wrong one — this is exactly how a mapping run was lost before.\n"
              "Stop it first:\n"
              "  ps aux | grep '[a]mcl\\|[m]ap_server' | awk '{print $2}' | xargs -r kill -9")
        sys.exit(1)
    if not n_slam:
        print("\nslam_toolbox is not running — nothing to save.")
        sys.exit(1)

    pubs = map_publishers()
    print(f"/map publishers: {pubs or '(none seen)'}")
    others = [p for p in pubs if p != "slam_toolbox"]
    if others:
        print(f"\nREFUSING TO SAVE: /map also published by {others}.\n"
              "The save reads the /map topic, so this is a coin flip. Note these\n"
              "may be on ANOTHER MACHINE on the same ROS_DOMAIN_ID, or a stale DDS\n"
              "endpoint left by a SIGKILLed node — a local `ps` will look clean.")
        sys.exit(1)

    os.makedirs(MAPS_DIR, exist_ok=True)
    base = os.path.join(MAPS_DIR, name)

    print(f"\nsaving occupancy grid -> {base}.pgm/.yaml")
    ok, out = save_grid(base)
    if not ok:
        print("map_saver_cli FAILED:\n" + out)
        sys.exit(1)

    print(f"saving pose graph    -> {base}.posegraph/.data")
    ok2, out2 = call_service("/slam_toolbox/serialize_map",
                             "slam_toolbox/srv/SerializePoseGraph",
                             f'{{filename: {base}}}')
    if not ok2:
        print("WARNING: pose-graph serialization failed; the grid was still saved.\n"
              "You will not be able to EXTEND this map later.\n" + out2)

    print("\nverifying saved map:")
    if not verify(base + ".pgm"):
        sys.exit(1)
    for ext in (".pgm", ".yaml", ".posegraph", ".data"):
        p = base + ext
        if os.path.exists(p):
            print(f"  {p}  ({os.path.getsize(p):,} bytes)")
    print("\nSaved and verified.")


if __name__ == "__main__":
    main()
