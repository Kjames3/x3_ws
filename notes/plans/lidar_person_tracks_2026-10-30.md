# Lidar-held person tracks with camera confirmation (2026-10-30)

Answer to the professor's 2026-10-22 feedback (people lost up close, id churn): 21% of
lab time had no person estimate, 139 of 354 dropouts started under 1 m (camera blind
range), 684 camera ids in 3 h with 9 people. The Livox sees 360° at 10 Hz down to ~0.1 m.

## Decisions (user, 2026-10-30)
- Camera seed: **C3** (`src/c3_live.py`), not v3. This ends C3's "never reaches the CBF"
  status, but only once phase 3 is switched on.
- Lidar-only hold cap: **3 s** (`lidar_person_tracker.HOLD_S`).

## Pieces
| Phase | What | Where | State |
|---|---|---|---|
| 0 | walk-up/crossing recordings | `scripts/record_person_lidar.sh` | to record in the lab |
| 1 | person-sized clusters of every full 10 Hz frame, in odom | `yahboomcar_bringup/person_candidates.py`, `livox_scan_node.publish_person_candidates` → `/livox/person_candidates` | built, offline-tested |
| 2 | camera-seeded tracker, lidar-owned ids, shadow (GUI + log) | `src/lidar_person_tracker.py`, `server_x3._person_candidates_cb`, GUI "Lidar-held people" | built, offline-tested, **on** by default (`X3_LIDAR_PERSONS=0` turns it off) |
| 3 | confirmed lidar tracks → PersonMemory/CBF where v3 has nobody | `lidar_person_tracker.cbf_estimates`, `_move_impl` | built, **off** (`X3_LIDAR_PERSONS_CBF=1` turns it on) |

Rules: birth only where a fresh C3 detection lands on a cluster (clusters alone never
make a person); a new C3 id landing on a track joins it; a track ends after 0.4 s with
no lidar support or 3 s after the camera last saw it; ambiguous matches and two tracks
in one cluster get no update. Known limit: a person within one 18 cm grid cell of a wall
or furniture is one component with it and is not a candidate.

## Lab protocol (robot parked, phase 3 off)
On the robot, after pulling:
```bash
cd ~/x3_ws && source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash
colcon build --packages-select yahboomcar_bringup yahboomcar_base_node
sudo systemctl restart x3_server
# after ~60 s, both should show up:
grep "person candidates" /tmp/ros2_bringup.log | tail -2      # N/frame, ms avg/max
journalctl -u x3_server -n 300 | grep "lidar-persons"         # births / re-links once someone walks past
```
Short trial first: `bash scripts/record_person_lidar.sh trial 20` while walking once past
the front; the replay summary at the end must show frames > 0 and at least one birth.

Then (60 s each unless noted):
1. `walkup-r1..r3`: start 4 m out in front, walk at the robot to ~0.5 m, stand 3 s, back off.
2. `crowd-r1, r2`: the same with 2-4 people standing behind you (the demo case).
3. `cross-r1, r2`: two people cross each other ~1.5 m in front.
4. `furniture-r1`: walk slowly past a chair/table edge at ~1-2 m.
5. `circle-r1`: walk a full circle round the robot at ~1.2 m (camera sees only the front).
6. `busy-r1` (300 s): normal lab traffic, nobody scripted.

Watch the GUI Surroundings panel while recording: green ring = camera + lidar, amber =
lidar-held, grey = tentative/coasting.

## What decides phase 3
From `lidar_person_replay.py` on each run: combined vs camera coverage, lidar ids vs
camera ids, held seconds, and false holds (amber rings on furniture/walls, read from the
GUI notes and the bag). Also the candidate timing line (target: well under 10 ms). Then a
walk-up with `X3_LIDAR_PERSONS_CBF=1` and Nav2 up, people behind, as in the demo.
