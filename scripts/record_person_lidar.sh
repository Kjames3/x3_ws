#!/usr/bin/env bash
# Record one lidar-person-tracker run ON THE ROBOT (x3_server up with the OAK/C3 and the
# Livox).  Phase 0 of the lidar-held person tracks (notes/plans/lidar_person_tracks_2026-10-30.md).
#
#   bash scripts/record_person_lidar.sh walkup-r1 60
#
# Writes artifacts/person-lidar-<date>/<name>/:
#   bag/                 /livox/lidar (raw, for re-clustering), /livox/person_candidates,
#                        odom, imu, scan, tf, ToF, cmd_vel_*, people obstacles
#   lidar_persons.ndjson every tracker frame (candidates + C3 tracks + live results),
#                        via the /tmp flag file server_x3 polls once a second
#   c3/c3.ndjson         every C3 camera packet: person boxes before the confidence cut
#                        and NMS (>= 0.2), the boxes kept, torso measurements, tracks
#   c3/c3_depth.bin      the depth frame of each packet (c3_live.read_depth_log)
#                        (both via /tmp/x3_c3_record; needs c3_live with the recorder)
#   server.log           the x3_server journal for the run
# Then: python3 src/tools/perception/lidar_person_replay.py <run>/lidar_persons.ndjson
set -euo pipefail
name="${1:?usage: record_person_lidar.sh <run-name> [seconds]}"; secs="${2:-60}"
cd "$HOME/x3_ws"; out="$PWD/artifacts/person-lidar-$(date +%F)/$name"
if [ -e "$out" ]; then echo "$out exists; pick the next name" >&2; exit 1; fi
unset ROS_DISCOVERY_SERVER FASTDDS_DEFAULT_PROFILES_FILE
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash 2>/dev/null
source ~/ws_livox/install/setup.bash 2>/dev/null; set -u
export ROS_DOMAIN_ID=42
flag=/tmp/x3_lidar_persons_record
c3flag=/tmp/x3_c3_record

topics=$(ros2 topic list 2>/dev/null)
for t in /livox/lidar /livox/person_candidates /odom /scan; do
    grep -qx "$t" <<<"$topics" || { echo "$t is not advertised; is the bringup (and the rebuilt livox_scan_node) up?" >&2; exit 1; }
done

mkdir -p "$out"
# Which arm of the CBF A/B this run is (server_x3 LIDAR_PERSONS_CBF_FILE; absent = env default, off).
cat /tmp/x3_lidar_persons_cbf > "$out/cbf_feed.txt" 2>/dev/null || echo default > "$out/cbf_feed.txt"
echo "lidar-person CBF feed: $(cat "$out/cbf_feed.txt")"
since=$(date '+%Y-%m-%d %H:%M:%S')
echo "$out/lidar_persons.ndjson" > "$flag"
echo "$out/c3" > "$c3flag"
trap 'rm -f "$flag" "$c3flag"' EXIT
echo "recording $secs s into $out ..."
# stdin from /dev/null: under `timeout` the recorder is a background job (see record_walk.sh).
timeout -s INT "$secs" ros2 bag record -o "$out/bag" \
    /livox/lidar /livox/person_candidates /odom /odom_raw /imu/data /scan /tf /tf_static \
    /cmd_vel /cmd_vel_nominal /cmd_vel_safe /pedestrian_states /people/obstacles \
    /tof/upper/obstacles /tof/lower/obstacles \
    < /dev/null > "$out/record.log" 2>&1 || true
rm -f "$flag" "$c3flag"
sleep 1.5      # the server closes the files on its next 1 s check
journalctl -u x3_server --since "$since" --no-pager -o short-precise > "$out/server.log" 2>/dev/null || true
ros2 bag info "$out/bag" | grep -E "Duration|livox|odom |person_candidates" || true
if [ -s "$out/lidar_persons.ndjson" ]; then
    python3 src/tools/perception/lidar_person_replay.py "$out/lidar_persons.ndjson"
else
    echo "WARNING: no tracker frames recorded: is x3_server running the new code (X3_LIDAR_PERSONS != 0) with C3?" >&2
fi
if [ -s "$out/c3/c3.ndjson" ]; then
    python3 - "$out/c3" <<'PY'
import json, sys
pk = [json.loads(l) for l in open(sys.argv[1] + '/c3.ndjson')]
pk = [p for p in pk if p.get('k') == 'pkt' and 'raw' in p]
if not pk:
    sys.exit('c3: no recorder packets (old c3_live on the robot?)')
mx = lambda k: max(len(p[k]) for p in pk)
print(f"c3: {len(pk)} packets, most per packet: raw boxes {mx('raw')}, kept {mx('dets')}, "
      f"measured {mx('meas')}, confirmed tracks {max(sum(t['confirmed'] for t in p['tracks']) for p in pk)}; "
      f"depth saved {sum(p.get('depth_rec', False) for p in pk)}")
PY
else
    echo "WARNING: no C3 packets recorded: is x3_server running c3_live with the recorder?" >&2
fi
echo "done: $out"
