# Progress log

Newest entries at the top. Each entry: **Problem**, **Potential solution**, **Work done**, **Results**.
Dates are when the work happened. Commits are on `main` unless a branch is named.

---

## 2026-11-23 — Rear OAK joins the lidar person tracker (X3_REAR_PERSONS)

**Problem.** Only the front, left and right cameras fed the tracker; a person behind the robot had no camera anchor.

**Potential solution.** Reuse the side-camera path: a person box on the GPU becomes a ray, and livox_scan_node finds the body along it.

**Work done.**
- `_rear_preview` adds a 1280x720 output. `side_person_loop` runs the same TensorRT yolo26n on it as role `rear`, so anchors are named `rear:<n>`.
- `side_person_rays` gained a pinhole/rational lens path. `config/oak_rear_rgb_intrinsics.json` holds the EEPROM values with fx/fy ×0.958, which matches today's TV solve at 1080p (1094.4 vs 1094.6).
- livox_scan_node listens on `/side/rear/person_rays`. Enabled on the robot with `X3_REAR_PERSONS=1` (f3a7ddf).
- **First live test:** the rear found the user (279 boxes per 60 s, 16 ms per detect), but the search window ended at 1.7 m for a person at 2 m. The rear camera frame had sagged about 10° up from loose mounting screws. The user re-tightened it back to about 2°.
- **Ray age 0.3 → 0.6 s** (c684cc0). The rear detects at about 3.3 Hz and the Livox runs at 10 Hz, so some lidar frames had no ray. A body found only through a ray then crossed the 0.4 s lidar gap and was reborn.

**Results.**
- **Crouched beside a table at 2 m:**
  - Before the ray-age change: a track was present in 51% of frames, with 5 ids in 20 s.
  - After: present in 94%, but still 4 ids in 30 s. The cluster is probably mixing with the furniture. Open.
- **Standing, then walking behind → left side → front → back to rear (75 s):** a track was present in 85% of frames, and one id (L24) held 64 s through rear, left, front and rear again. Its camera ids were `rear:1/3/4/5`, `left:1/2` and front C3 `0/1`.
- **Leftovers:** five 1–4 s duplicate tracks while L24 was lidar-held near the front. Front C3 anchors started their own track instead of joining L24. That's the next thing to fix: check for an existing track before a birth.

## 2026-11-23 — Slate AX router on the robot; GUI "HQ Stream" profile

**Problem.** GUI streams were sized for the school's congested WiFi 4 (15 fps, JPEG 70, small frames, 12000-point Surroundings cloud). The new GL.iNet Slate AX now carries the laptop–robot traffic locally.

**Potential solution.** Put the Orin and the Livox on the Slate's LAN ports, then add a live switch between the old settings and a high-quality profile for the local link.

**Work done.**
- Slate LAN set to 192.168.1.1/24, DHCP .200–.250; Orin `eno1` (static .50) in LAN1, Livox (.192) in LAN2, WAN empty; Slate repeats the school WiFi. Orin network config unchanged (its internet stays on UCR-SECURE). Powered from the USB hub for now; that port is limited to 1.5 A, so a dedicated 5.1 V buck is planned.
- Checked: Livox ping ~2 ms, `/scan` 10 Hz, `/imu/data` 200 Hz. The laptop on the Slate WiFi reaches `orin` over Tailscale *direct* via 192.168.1.50, at 322 Mb/s (SSH bulk transfer).
- 27418c1, 171773b: `--stream-profile low|high` and a GUI **HQ Stream** toggle (server-global, synced through the readout). 'high' = 30 fps, JPEG 85, 480p primary camera, 960x600 rear OAK, 24000-point Surroundings (`livox_scan_node` `display_cap` is now a live parameter). The net-tile thresholds follow the profile. New `src/tools/viewers/stream_bench.py` measures per-camera fps, frame size and Mb/s at the client.

**Results** (30 s each, laptop on Slate WiFi, same scene):

| | low (old) | high |
|---|---|---|
| Total WebSocket | 7.7 Mb/s | 23.4 Mb/s |
| front OAK 640x400 | 15.3 fps, 26 KB | 21.4 fps, 43 KB |
| rear OAK | 480x300, 14.2 fps | 960x600, 24.9 fps |
| left 640x400 | 15.6 fps | 29.1 fps |
| right (primary) | 384x240, 16.2 fps | 768x480, 14.2 fps |
| frame gap p95 | 105–122 ms | 76–112 ms |
| Surroundings points | 12000 (capped) | ~15300 (uncapped scene) |
| x3_server CPU | ~484–516% | ~520–550% |

- Gains: 3x bandwidth for sharper, larger frames; left camera now at its full 30 fps; rear at 4x the pixels; Surroundings no longer truncated (~28% more points).
- Costs: ~+0.4 cores on `x3_server`; still only 7% of the measured link capacity.
- Not reaching 30 fps: front OAK (~21, the driver delivers ~27–28) and right (~14, limited by the primary camera loop). Still open.
- Bugs found and fixed while testing: the first cap change after a restart was dropped (the parameter service wasn't discovered yet; it now retries for 15 s), and a 20 ms poll held 'high' to 20–24 fps.
- Default stays `low`: on school WiFi the old limits still apply. Switching profile reopens the rear OAK (~2 s USB re-enumeration). If `livox_scan_node` restarts, the cap falls back to 12000 until the toggle is flipped again.

## 2026-11-23 — Rear OAK stereo recalibrated on the 55" TV

**Problem.** The rear OAK-D Pro W (14442C103181D7D600) gave wrong stereo depth.

**Potential solution.** Recalibrate it against a ChArUco board shown full-screen on the apartment TV, without the Luxonis `calibrate.py` (which needs depthai v2 and a desktop).

**Work done.**
- The board was 11x8 squares, DICT_4X4_1000, marker 0.75 of the square, square 7.5 cm (caliper).
- Captured 40 synced left/right/RGB views with `src/tools/calibration/oak_tv_charuco_capture.py`.
- Solved with `oak_tv_charuco_solve.py` and flashed with `oak_tv_charuco_flash.py --stereo-only`.
- Backups of the user and factory EEPROM are in `~jetson/calib_backup/` and `artifacts/rear-oak-tv-calib-2026-11-23/`. Restore with `--restore`.

**Results.**
- **Old calibration:** triangulating the board with it made each 7.5 cm square 8.72 cm, so depth read about 16% far.
- **Cause:** the stored left→right yaw was 0.46° off, a disparity shift of about 4.6 px.
- **Full re-solve, not flashed:** the board covered only the image centre (radius ≤ 391 px), and the free rational lens model drifted 30–58 px from the factory one at the edges.
- **What was flashed:** the factory lens values fit the views at 0.19 px rms, so they were kept, along with the RGB calibration. Only the left→right extrinsics and rectification were replaced: baseline 7.49 cm, triangulated square 7.51 cm, rectified row error 0.16 px median.
- **Live check:** on-device StereoDepth at the board matched the board's own distance to within 1.3% (ratio 1.013, ~1 m, 219 frames).
- **2 m check:** tape 2.0 m, board distance from a single camera 1.99 m, stereo/board ratio 1.001 (only 7 frames with the board detected).
- **3 m check:** tape 3.0 m, median stereo depth over the board 3.008 m, 291 frames, p10–p90 3.008–3.015 m. The mono cameras can't read the markers at 3 m, so this was measured over a fixed box on the screen.

## 2026-11-22 — Side cameras live: seated runs, left and right

**Problem:** first live test of X3_SIDE_PERSONS: can a seated person seen only by a side camera be tracked?

**Work done:** robot parked; the user sat working at a desk on the left (`side-sit-left-r1`, 120 s), then on the right (`side-sit-right-r1`, 120 s). Analysed the recorder summary and the live tracker output (`lidar_persons.ndjson`) and checked a recorded side JPEG.

**Results:**

| run | rays | ray found a body | agrees with the track | live confirmed track | ids |
|---|---|---|---|---|---|
| left | 1172 | 98.1% | 98.1% | **100%** of 1191 frames | 1 (L1) |
| right | 1121 | 98.0% | 98.0% | **99.0%** | 1 (L2) |

- No frames had a second track.
- Off-person bearings that found a "body": 7.2% (left) and 9.8% (right).
- The JPEG shows the user seated at the desk, side-on.
- The recorder's "camera coverage 0% / births 0" summary is the front-camera (C3) offline replay and does not include side anchors; nobody was in front of the robot.
- **Next:** courtyard walks past each side, standing at taped 1/2/4 m, and an empty run for side false births.

## 2026-11-22 — Side-camera tracking merged and enabled on the robot

**Problem:** the side-camera branch had to be merged with today's id revival and set up on the AGX.

**Work done:**
- Merged `t3/side-camera-livox-tracking` (a08485c). The conflicts in the tracker stats, the log line, the tests and PROGRESS were resolved keeping both sides. 65 tests pass (tracker + all side-camera tests).
- Robot:
  - colcon-built `yahboomcar_bringup`;
  - rebuilt `yolo26n.engine` on the AGX with trtexec `--fp16` from `yolo26n.onnx`. The old engine warned "built on a different device"; it is kept as `.2026-11-20-bak`;
  - added `X3_SIDE_PERSONS=1` to the x3_server override (with `X3_SEEDED_TRACKER=1`; the CBF feed is still off);
  - restarted x3_server (which launches the bringup, i.e. the scan node) and foxglove_bridge.

**Results:**
- The new engine runs at 8.7 ms/frame with no device warning.
- The log shows `[side-persons] TensorRT detector ready` and `side-camera people ON for left, right`.
- `/side/left/person_rays` and `/side/right/person_rays` publish at ~4.8 Hz.
- `/livox/person_candidates_side` is silent with nobody beside the robot. Live recordings with people are next.

## 2026-11-22 — Lidar person tracker: revive recently ended ids

**Problem:** live walk run: 15 confirmed ids in 2 min. Most restarts came 0.2–2 s after a track ended, 0.1–1.5 m from it, often with the same C3 camera id. The C3 ids themselves also change between walks.

**Potential solution:** a birth within 4 s of a track ending takes over that track's id. It qualifies if it carries one of the old track's camera ids, or starts within 0.8 m of where the old track would be by now (velocity extrapolated ≤ 1 s).

**Work done:** `LidarPersonTracker` keeps recently ended confirmed tracks (`self.ended`). Births call `_revivable()`, and the new `stats['revived']` counter is in the server log line. Revived tracks go through the 3-hit confirmation again. Three tests (same camera id far away → same id; new camera id nearby → same id; new camera id far away → new id); 28 pass. `seeded_ray_replay.py --no-revive` gives the baseline.

**Results (replay, live config, confirmed ids):**

| run | before | after |
|---|---|---|
| live walk | 14 | 11 |
| live desk | 12 | 7 |
| shadow walk | 5 | 5 |
| empty | 0 | 0 |

- Coverage is unchanged (walk 94.4%, desk 98.0%).
- Most remaining new ids follow gaps > 4 s, with the person out of the front camera's view. Side-camera tracking (in progress on a separate branch) is the fix for those.

## 2026-11-22 — Side cameras feed the lidar person tracker (X3_SIDE_PERSONS, branch t3/side-camera-livox-tracking)

**Problem:** only the front camera (C3) can start a lidar person track. Someone beside the robot, seen only by the left or right mono camera, never becomes a track.

**Potential solution:** a side-camera person box gives a bearing but no range. Search the whole Livox ray for the nearest person-sized upper body, and use that body both as a candidate and as the camera anchor.

**Work done:**
- `person_candidates.ray_candidates()`: searches the ray from 0.4 to 6 m, or a window from the box, and takes the nearest body. A wrong pick becomes a wrong person, so it rejects structure itself:
  - the object runs at most 1 m along the ray;
  - it does not continue sideways at its depth;
  - its top is 1.0–1.95 m (lower up close, where the Livox FOV cuts the head);
  - n·s² ≥ 300;
  - its top matches the head height implied by the box top, when the box top is in the image.
- `livox_scan_node` subscribes `/side/{left,right}/person_rays` and publishes `/livox/person_candidates_side` (FIELDS + ax, ay, cam, tid, age).
- `LidarPersonTracker.update(side=...)` takes side anchors with ids `left:<n>` / `right:<n>`. They bypass the C3 packet gate, are not snapped, do not pull velocity, and are dropped within 0.5 m of a C3 anchor (two anchors on one track would block each other's confirmation).
- `side_person_rays.py` turns a box into a ray (fisheye K, D, extrinsics from /tf_static) with a coarse range window from the feet and a 2 m person-height bound, and keeps per-camera IoU ids, ready after 2 sightings.
- `server_x3` with `X3_SIDE_PERSONS=1` (default off) runs a dedicated yolo26n TensorRT detector on each side camera at 5 Hz, confidence ≥ 0.4; with no engine the feature is off.
- Recording: `side/side_rays.ndjson` plus every 5th frame as JPEG. `record_person_lidar.sh` bags the new topics.
- `side_ray_eval.py` replays recorded side rays. On older runs it casts each C3 person as a bearing-only ray instead.
- 35 new tests: ray search, config-file geometry, tracker side anchors, scan node, server wiring.

**Results** (no bag has side-camera images yet, so this is the same lidar code on the 11-22 runs, C3 people as bearing-only rays, range discarded, no box top; reference = the live confirmed track on the ray):

| run | rays | body found | agrees with track | wrong body | missed | person-free bearings that found a "body" |
|---|---|---|---|---|---|---|
| seededlive-desk | 2380 | 100% | 100% | 0% | 0% | 5.6% |
| seededlive-walk | 759 | 82.5% | 77.0% | 8.9% | 14.1% | 29.6% |
| seeded-walk | 179 | 94.4% | 96.0% | 0% | 4.0% | 9.4% |
| seeded-couch | 818 | 8.4% | (24 with a track) | | | 21.3% |
| empty runs (3) | – | – | – | – | – | 3.0–21.9% |

- The top and area rules were **fitted on these runs**. They took the person-free picks from 22–61% to 3–30% and kept 99.8% of the bodies that match a track.
- **Held out** (not used to fit):

  | run | rays | found | agree | wrong | missed | person-free picks |
  |---|---|---|---|---|---|---|
  | 11-20 lab walk-r1 / r2 | 47 / 51 | 94% / 92% | 100% / 83% | 0 / 6% | 0 / 11% | 40% / 41% |
  | 11-20 lab seated-r1 / r2 | 552 / 576 | 100% | 100% | 0 | 0 | 26% |
  | 11-20 lab empty-r1 | – | – | – | – | – | 51% |
  | 10-30 walkup-r1 / r2, circle-r1 (old chassis, people walking up close) | 350 / 347 / 159 | 100% | 100% | 0 | 0 | 46–54% |

  On-person picks hold, including close walk-ups, where the Livox FOV cuts the head. But in the lab and on the old runs, half of the person-free bearings still find something person-shaped. **The ray search is only as good as the side-camera detection that starts it.**
- Side rows are accepted one Livox frame late (0.15 s): on the live desk run ~30% of seeded rows arrived after their candidates message. Side anchors are also deduplicated among themselves.
- The walk run's wrong and missed rays are one parked track at ~3.95 m. The ray takes a body 0.8–1.3 m nearer, and it is not clear which is the walker.
- Couch-seated people are mostly not found, as with the classifier.
- Shape alone cannot reject everything: the side camera's detection (confidence 0.4, 2 sightings) and the box's range window are the rest of the protection.

**Next (robot):**
- colcon-build `yahboomcar_bringup` (scan node and person_candidates).
- Check that the AGX has `yolo26n.engine`, and that the Arducam is not held by `--webrtc-camera`.
- Run with `X3_SIDE_PERSONS=1` (plus `X3_SEEDED_TRACKER=1`) and record walks and sits on the left and right.
- Keep `X3_LIDAR_PERSONS_CBF` off with this flag until validated: a side-born track confirms itself.
- Untested anywhere: the detector loop, YOLO on these mono fisheye frames (TRTDetector stretches 640×400 to 640×640), the recorder and the eval tool's side mode.

## 2026-11-22 — Line-of-sight tracker, live lab runs (X3_SEEDED_TRACKER=1)

**Problem:** do the seeded candidates and ray association hold up live over minutes, and do they add false tracks?

**Work done:** recorded `seededlive-desk-r1` (300 s: working, leaning, turning, standing up once), `seededlive-walk-r1` (120 s: crossings at 1.5/3 m, close approach, partial occlusion, sitting at the end) and `seededlive-empty-r1` (90 s, user left at the start). Analysed the live tracker output in `lidar_persons.ndjson`.

**Results:**

| run | camera frames | confirmed track | confirmed ids | longest id |
|---|---|---|---|---|
| desk | 2665 | **98.2%** | 15 | 196 s (then 45 s, 18 s) |
| walk | 857 | **95.2%** (shadow walk earlier: 77%) | 15 | 21 s |
| empty | 14 (user leaving) | 1 frame, at t≈0 while leaving | 1 | 0 s |

- **Desk:** three ids cover ~258 of the 300 s; the other 12 lasted under 3 s. Most uncovered frames cluster at t≈18–30 s.
- **Walk:** ids churn: 15 in 2 min, longest 21 s.
- **Empty:** no false tracks after the user left.
- 29 frames per run had two confirmed tracks at once (a duplicate or a second object).
- **Next:** id continuity on walks (relink after a loss) and the short-lived extra ids; then consider X3_LIDAR_PERSONS_CBF.

## 2026-11-22 — Line-of-sight tracker: first live check at the lab desk

**Problem:** confirm live that a seated desk user keeps a lidar track with X3_SEEDED_TRACKER=1.

**Work done:** the user sat at their desk and watched the GUI Surroundings view; I checked the server log.

**Results:**
- The user reports the track is solid and has not dropped.
- Log: 9 births and 8 lidar-lost endings in the first ~70 s after the 16:27 restart (people moving around, settling in). No new births or endings after 16:28:21, so a single track has held since. 0 tracker errors.
- The CBF feed is still off. Next: longer desk/walk runs with the flag on, then consider turning on X3_LIDAR_PERSONS_CBF.

## 2026-11-22 — Line-of-sight camera association in the live tracker

**Problem:** the camera over-reads seated people by ~0.7 m, so it could not confirm desk tracks (0.55 m gate).

**Potential solution:** snap each camera anchor onto the nearest candidate along its ray before association (offline test: desk 32% → 96%).

**Work done:**
- `lidar_person_tracker.snap_to_ray()` (lateral 0.5 m, 1.2 m nearer to 0.5 m beyond). `update(..., origin=None)` uses it when an origin is given.
- `server_x3.py` passes the /odom robot XY as the origin when `X3_SEEDED_TRACKER=1`. One flag now covers seeded candidates and ray association.
- Two tests (snap window; an over-read camera confirms a track only with an origin). 25 pass. `seeded_ray_replay.py` now uses the tracker's own code.
- Deployed with `X3_SEEDED_TRACKER=1` in the x3_server override. The CBF feed stays off.

**Results (replay, confirmed track on the person's ray):**

| run | base | live config |
|---|---|---|
| desk | 20.2% | **96.1%** |
| walk | 76.8% | 81.9% |
| couch | 3.6% | 98.7% |
| both empty runs | 0 | 0 |

- Births: desk 12 (was 21 at lateral 0.35, 27 seeded-only), 6 ids.
- Live check in the lab still pending.

## 2026-11-22 — Camera↔track association along the camera ray (offline test)

**Problem:** at the desk, the camera reads the seated person ~0.7 m too far away. That is beyond the tracker's 0.55 m camera↔track gate, so seeded tracks lose their camera confirmation and restart.

**Potential solution:** move each camera anchor onto the nearest candidate along its ray (lateral < 0.35 m, from 1.2 m nearer to 0.5 m farther than the camera range) before the tracker sees it.

**Work done:** `src/tools/perception/seeded_ray_replay.py` replays a run three ways: base, +seeded, and +seeded+ray. The robot origin comes from the bag. Also fixed the import path in `seeded_tracker_replay.py`.

**Results** (confirmed track in camera frames; births):

| run | base | +seeded | +seeded+ray |
|---|---|---|---|
| desk | 21.4% (9) | 32.4% (27) | **96.3% (21)** |
| walk | 76.8% (4) | 80.2% (5) | 81.0% (5) |
| couch | 3.6% (1) | 99.0% (3) | 99.0% (3) |
| lab empty / couch empty | 0 confirmed | 0 | 0 |

- **Desk tracks are on the person:** all lie within 0.5 m of the camera ray (median 0.36 m to the side) and a median 0.64 m nearer than the camera range, which is the over-read. The "on the ray" metric in the script uses the 0.35 m cut and reports only 39%, which undercounts.
- The consistent ~0.36 m sideways offset suggests a small C3 bearing bias at the desk. Widen the lateral window to 0.5 m.
- **Still open:** the desk has 21 births / 9 ids in 90 s (id churn). Nothing is live yet. Next is to put the ray association in the tracker/server behind the same flag and test it live.

## 2026-11-22 — Seeded tracker, lab desk / walk / empty runs (replay)

**Problem:** does feeding seeded candidates into the tracker (X3_SEEDED_TRACKER) also help in the lab, and does it add false tracks?

**Work done:** recorded `seeded-desk-r1`, `seeded-walk-r1` and `seeded-empty-r1` in shadow mode. For the empty run, the user left the room at the start. Ran `seeded_tracker_replay.py` and `camera_seed_eval.py`; seeded-empty-r1 was added to EMPTY_RUNS. Output: `artifacts/person-lidar-2026-11-22/seeded_lab_eval.txt`.

**Results:**

| run | camera frames | base: track on person | seeded: track on person | births (base → seeded) |
|---|---|---|---|---|
| desk | 868 | 0.5% (21% any track) | 3.2% (32% any track) | 9 → 27 |
| walk | 237 | 68.4% | 71.3% | 4 → 5 |
| empty | 13 (user leaving) | 0 | 0 | no confirmed track |

- **Walk:** a small gain and no harm.
- **Empty:** no false tracks.
- **Desk:** the seeded search finds a body on the camera ray in 100% of instances. But in 406 of 703 it is suppressed as a duplicate of an ordinary candidate (the desk/chair cluster), and the body is a median **0.73 m** from the camera XY. That is the known camera range over-read on seated people. It is beyond the tracker's 0.55 m camera↔track gate, so the camera rarely confirms the track it found. Tracks start, lose their camera confirmation and die, which gives 27 births.
- The classifier scores 72% of the new desk rows above 0.5 (trimmed). Unlike the couch, the desk body does look like a person.
- **Next:** association has to use the camera ray, not the camera XY. Either snap each camera anchor to the body the seeded search found on its ray, or match camera to track by lateral offset from the ray plus a range window.

## 2026-11-22 — Auto-Explore stuck recovery; OAK floor threshold back to 3 cm

**Problem.** In the first lab run, 8 of 17 Auto-Explore goals ended in 30 s no-progress TIMEOUTs. With the 12 mm OAK threshold an extension cord became lethal, so the robot stalled at it and then drove over it anyway. The user is fine driving over cords; power strips should be avoided.

**Potential solution.** Put the OAK threshold back to 3 cm. Detect "commanded but not moving" early, back out, and keep the planner off that spot for a while.

**Work done.** OAK floor threshold 3 cm, +1 cm per metre beyond 1 m (0f47601). Stuck = Nav2 commanding at least 0.05 m/s or 0.25 rad/s for 4 s while the SLAM map pose moves less than 4 cm and turns less than 0.15 rad. Recovery cancels the goal, backs out about 0.3 m opposite the commanded direction through the CBF motion queue, and drops a 0.15 m disc 0.42 m ahead on `/explore/keepout` (map frame, new mark-only costmap source). The frontier counts as a failed try. First live run: the disc trapped the robot. It came back to the stuck spot, and the disc was inside its 0.44 m turning radius, so every goal aborted "obstacle too close". Fix (67f166d): discs expire after 90 s and are dropped within 0.6 m of the robot after any failed goal. 2 new explorer tests; 13 pass.

**Results.** The cord 0.5 m out peaked at 3.1 cm, at the threshold. Bare carpet stayed under 1 cm. First run: one STUCK fired with back-out and disc, then 4 ABORTED (the trap above). After the fix: the first 6 goals all succeeded, with no TIMEOUT or STUCK, on a fresh SLAM map after the restart. The power strip has not been placed in front of the camera yet, so avoidance of it is untested.

**Lab check.** Put the power strip 0.5–1 m ahead and confirm it is marked. Watch the next STUCK event to confirm the back-out moves the robot backward.

## 2026-11-22 — Front-OAK floor obstacles (power strips, extension cords)

**Problem.** A power strip and an orange extension cord sit below the Livox low band (0.06 m) and are only lidar-visible from ~2 m, so Auto-Explore could drive into them.

**Potential solution.** Use front-OAK stereo depth within ~1.35 m of the bumper, measure height above a floor plane fitted in each frame, and remember marks in odom like the Livox band.

**Work done.** `src/oak_floor_obstacles.py`, started by server_x3, publishes `/oak/obstacles` (odom, 4 cm voxels, at least 2 returns per cell). It is the mark-only `oak_low` source in both costmaps (088889a, 8f7878f, 9ccdeac, 16d045e). With the URDF floor height the lab carpet read +5.7 cm: the fitted plane sits 7.3 cm above the URDF floor, likely because B1F4 depth is uncorrected. The plane fit is flat to ±5 mm. The threshold is 12 mm plus 1 cm per metre of depth beyond 1 m. Marks are made only where the camera can also clear them. Before that fix, people standing at 1.6 m stayed marked for 30 s. Also fixed: `ValueError` from an empty `asyncio.wait` on every x3_server shutdown.

**Results.** 1 m past the bumper, the cord read no higher than carpet (2–4 mm; depth steps there are about 5 cm). 0.5 m past the bumper it read 13–31 mm. It was marked, and the local costmap had cost 100 at 0.80–0.95 m. Auto-Explore then ran to completion: 9 goals succeeded and 8 timed out, with no aborts. Map saved as `RealLab4_explore_2026-11-22`, with posegraph. The robot's path was not logged, so I can't confirm here whether it went near the cord.

**Lab check.** Send one goal just past the cord and watch it route around. Find what causes the TIMEOUT stalls. Depth only covers ±15 cm to each side near the bumper. A colour detector for cables is the next step if thin cords are still missed.

## 2026-11-22 — Livox low band into the costmaps; laptop_viz on the AGX

**Problem.** In the new lab, Auto-Explore hit table and truss bases and caught on cables. Nav2 and the CBF see only `/scan`, a 0.26–0.36 m slice of the Livox cloud, and the ToF arrays are off the chassis. Separately, `laptop_viz.sh` found no topics on the AGX.

**Potential solution.** Publish the cloud's 0.06–0.45 m band as a costmap source and remember it inside the lidar's blind radius. Make `laptop_viz.sh` join the AGX discovery server.

**Work done.** `livox_scan_node` publishes `/livox/obstacles` in `odom`. It remembers 5 cm voxels until their spot is back in view (-7° lowest ray) and empty, for at most 20 s, so `/scan` raytracing can't erase a bar the lidar no longer sees. It is a mark-only `livox_low` source in the local and global costmaps (e200e9b, bf0df38). The floor cutoff is 0.06 m plus 1.5 cm/m beyond 2 m: parked on lab carpet, floor returns peaked at -0.02–0 and 0.02–0.05 m out to 4.5 m. `laptop_viz.sh` probes robot:11811 and becomes a discovery-server client (03a59dc). Plan for cables: `notes/plans/low_obstacle_and_cable_detection.md`.

**Results.** Parked in the lab, `/livox/obstacles` held about 1,450 points: walls, cabinets, and chair and truss bases, with no floor speckle. Nav2 was not running, so the costmap and driving are untested. The CBF does not use this source yet. From the laptop over Tailscale, `/odom`, `/scan`, `/tf`, `/map` and `/robot_description` arrived. `/livox/lidar` did not.

**Lab check.** Run Auto-Explore near the truss and the chair bases. Watch the local costmap in `laptop_viz.sh orin nav` and confirm low bars stay marked as the robot closes in.

## 2026-11-22 — Auto-Explore stop handling, frontier selection and simulation

**Problem.** The operator could stop the wheels while Nav2 retained a goal. The explorer could mark a failed approach as visited or aim at a centroid across a wall. Mapping and saved-map localization could also launch simultaneously from the GUI.

**Potential solution.** Give each navigation action an identity, inhibit motion throughout cancellation, and select reachable approach cells using a fresh map and map-frame pose.

**Work done.** Global Stop latches movement off. Stop Auto-Explore cancels its owned goal. The selector scores known-free approaches by grid travel cost and estimated information gain, checks footprint clearance and visibility, and runs on a worker with a map-revision cache. The explorer retries failed goals, distinguishes unresolved frontiers and monitors progress, map age and TF freshness. GUI SLAM Mode starts one mapper with navigation-only Nav2. Gazebo now receives gated velocity and supplies a usable simulation clock, odometry TF, laser frame and bounded room.

**Results.** On this workstation, 354 focused tests and the simulated Nav2 action lifecycle harness passed. A headless Fortress run used the GUI WebSocket commands, SLAM Toolbox, Nav2 and the server velocity gate. In a 90-second mapping run, known cells grew from 5,608 to 24,730. Four frontier goals succeeded. After a planner abort, the explorer selected another goal and left the visited count unchanged. Global Stop canceled the active goal and all 26 sampled driver commands during continued nonzero input were zero. A separate run verified dedicated Stop, explicit explorer restart, then global Stop. The CBF reported infeasible constraints during a close obstacle approach. A fresh mapping session started at the previous obstacle-side stopping point waited for mapped clearance around the base. Start the lab trial on open floor. Physical stopping distance, clearance and whole-room completion remain unverified.

**Lab check.** Build the changed ROS packages on the Jetson and load the updated server. Select SLAM Mode, Launch Nav2, then Start Auto-Explore. With room to stop, verify Stop Auto-Explore cancels the action and Global Stop holds the base still until Enable Movement. Then observe one doorway or obstacle approach, record minimum physical clearance and map growth, and check the displayed reason for any blocked frontier. Hardware deployment is pending.

## 2026-11-22 — Seeded candidates into the tracker (item 1), behind X3_SEEDED_TRACKER

**Problem:** seated people on the couch got a lidar track in only ~3% of camera frames. The classifier cannot confirm seeded couch clusters (see the height-cap entry), so they can't be gated on shape.

**Potential solution:** feed the camera-seeded rows into `LidarPersonTracker` as extra candidates. The tracker already requires a camera detection on a cluster to start a track, so nothing new can appear without the camera.

**Work done:**
- `merge_seeded()` in `src/lidar_person_tracker.py`: adds seeded rows from the frame (≤ 0.15 s old), dropping any within 0.5 m of an ordinary candidate. The input list is not modified.
- `server_x3.py`: `X3_SEEDED_TRACKER=1` turns it on (off by default). The startup log says `[seeded] ... FEED` or `shadow`.
- Tests: merge/dedup/stale, and a seeded-only track ends once the camera stops (the lidar gap of 0.4 s). 23 pass.
- `src/tools/perception/seeded_tracker_replay.py` replays a recorded `lidar_persons.ndjson` both ways.

**Results (replay):**
- Couch seated, 838 camera frames: confirmed track on the person **2.9% → 98.7%**, 3 track ids (births) in 90 s.
- Empty couch: 0 tracks either way.
- Not yet tested live. Camera false positives would now turn into tracks, so check the lab empty runs before turning on the CBF feed.

## 2026-11-22 — Height cap on seeded couch clusters (item 2): does not help

**Problem:** the classifier accepted only 0.6% of seeded couch clusters, and those clusters reach ~1.97 m. Is the couch back or wall behind the person the reason?

**Potential solution:** drop the points above a cap before computing features, then rescore.

**Work done:** `src/tools/perception/seeded_zcap_check.py` caps at 1.1–1.6 m. As a control, it seeds the EMPTY couch run at the person's median position (1.25, -2.70) every frame.

**Results** (share of clusters scoring > 0.5):

| cap | occupied couch (798) | empty couch (438) |
|---|---|---|
| none | 0.6% | 0.0% |
| 1.6 m | 75.3% | **96.1%** |
| 1.4 m | 1.1% | 13.9% |
| 1.3 / 1.2 / 1.1 m | 0.6–0.9% | 0.0% |

- The point profile is almost the same with and without the person: about 20 points in every 0.2 m band from 0.4 to 2.0 m. Something tall stands right there (wall/couch back), and the seeded search finds it even when the couch is empty.
- The 1.6 m jump is the classifier reacting to standing-person height. The empty couch scores *higher* than the occupied one, so no cap separates person from furniture.
- **Conclusion:** at this spot, the lidar shape adds no confirmation. Retraining (item 3) would have to learn from geometry that doesn't differ either, so it is unlikely to work here. The viable route is item 1: a camera detection plus a seeded cluster becomes a track, with the lidar supplying the position. The camera must stay the gate, so camera false positives matter (lab empty runs).

## 2026-11-22 — Classifier scores on the seeded couch clusters

**Problem:** seeded candidates covered 86% of the seated frames, but their clusters reach about 1.97 m high. Does the classifier see a person in them?

**Potential solution:** score them with model_base_1121.joblib, using the offline evaluator `camera_seed_eval.py`.

**Work done:** replayed both couch bags through the evaluator (`--jobs 2`). Added `seeded-couch-empty-r1` to EMPTY_RUNS. Output is in `artifacts/person-lidar-2026-11-22/seeded_couch_eval.txt`.

**Results:**
- Seated run (818 camera-person frames): the base extractor matched you in 2.0% of frames. Seeded search found a body on the camera ray in 100%, adding a new row in 97.6% (median 0.05 m from the camera XY).
- **The classifier scored only 0.6% of the 798 seeded rows above 0.5 (0.4% on box features).** So it rejects seated-on-couch clusters almost always.
- Empty run: 0 seeds, 0 false alarms; the base extractor made 0 false alarms too.
- Runtime: seeded search 0.27 ms per frame.
- **Conclusion:** the camera finds the person and the lidar finds a body there, but the classifier does not confirm it. The model was trained mostly on standing people and desk sitters, so a body merged with the couch back does not look like a person. Options: (1) trust camera + seeded body without the classifier (a camera-gated track); (2) retrain with seeded couch rows as positives; (3) crop the cluster at a seated height before featuring.

## 2026-11-22 — Nav2 launch from the GUI sometimes loaded no nodes
- **Problem:** twice in one evening "Launch Nav2" left `nav2_container` running with no AMCL, map server, planner or controller, and no error anywhere (the server sent Nav2's output to /dev/null). The GUI still looked localized from the previous session.
- **Potential solution:** the launch logs in `~/.ros/log/<stamp>/launch.log` showed the bad launches stuck waiting on `/nav2_container/_container/load_node` (no response, ever), while good ones loaded 11 nodes in seconds: the composition service lost a request or reply during discovery over the FastDDS discovery server. Run Nav2 without composition.
- **Work done:**
  - `x3_nav2.launch.py`: `use_composition` argument, default **False**, passed to nav2_bringup (11 separate processes, no load_node service).
  - `nav2_client.py`: Nav2 output goes to `~/.ros/log/x3_nav2_launch.log` (overwritten each launch). Nav2 runs in its own process group; stop sends SIGINT to the group, then SIGKILL after 10 s, so no node outlives a stop and doubles up with the next launch.
  - `server_x3.py`: the GUI's stop_nav2 runs in an executor, so a slow shutdown no longer blocks the event loop (and the /cmd_vel heartbeat).
- **Results:** 4 launch/stop cycles through the server's WebSocket on the robot: AMCL and bt_navigator active within 2 s each time, 11 Nav2 processes, 0 left after each stop. Not yet seen over many sessions; the failure was intermittent.
  - Open: on launch the server seeded the initial pose at (0.25, 0.29), near the doorway runs' start, while the robot had been left at the goal; check the GUI arrow after every Nav2 launch and set the pose by hand if it is wrong.
- Also: the last doorway bag (`doorway_20261122_022333`) crashed with "database is locked" because the bag was read while still recording; reindexed, it ends before the successful run. Never open a bag that is still being written.

## 2026-11-22 — Doorway, part 2: no deadband with closed-loop wheels; map doorway widened
- **Problem:** after the footprint/margin change the robot got through the 0.88 m doorway, but crawled at 0.02–0.05 m/s for ~15 s, and the next run stalled at the threshold ("Starting point in lethal space"), backed up and spun before passing.
- **Potential solution:** replay the bags through the CBF to find what cut the speed; compare the saved map's doorway with the lidar's.
- **Work done:**
  - `CMD_DEADBAND_MPS` default 0.14 -> **0.0**. The 0.14 m/s is the open-loop driver's min_pwm; the bringup runs closed-loop wheel speed control (`speed_control: True`), which executes the command. Replaying bag `doorway_20261122_020804`: the CBF's own bound through the door was 0.14–0.19 m/s, minus 0.14 = the 0.00–0.05 m/s sent. `X3_CMD_DEADBAND_MPS=0.14` restores it for the open-loop driver; the deadband tests now set it explicitly.
  - `newApartment1.pgm`: cleared one jamb cell each side (19 cells, doorway rows y -0.93..-1.78 only). The map's opening was ~0.65–0.70 m; the lidar measures 0.80–0.86 m in it. Original kept in `artifacts/doorway-2026-11-22/newApartment1.orig.pgm`.
  - Reverted the global costmap resolution to 0.05: the static layer resizes the global costmap to the map's resolution, so 0.025 never took effect (correction to the entry below).
- **Results:**
  - Deadband fix (bag `doorway_20261122_021503`): through the door at 0.15–0.26 m/s, no crawl; the stall that run was the planner (map too narrow).
  - With the map edit (bag `doorway_20261122_022333`): goal **SUCCEEDED in 16 s**, no replanning failures or recoveries.
  - Two attempts before that one aborted on a lidar cluster in the doorway (~1.5 m ahead, 0.2–0.4 m wide) with nobody there; it was gone a few minutes later. Cause unknown.
  - Open: the GUI's Nav2 launch twice came up with an empty container (no AMCL/planner); the same launch by hand loaded everything. Nav2 output goes to /dev/null, so the reason is not logged.

## 2026-11-22 — Doorway: x3_v2 robot now drives through the 0.88 m apartment doorway
- **Problem:** Nav2 could not take the robot through the 0.88 m (measured) doorway into a room.
- **Potential solution:** find the blocker from a bag first, then shrink the clearances that the x3_v2 geometry does not need.
- **Work done:**
  - Bag 1 (`~/bags/doorway/doorway_20261122_014046` on the robot) was useless for the door: AMCL started at the map origin (no pose set, auto-pose ambiguous), so the robot drove its plan in the wrong frame. A second attempt found the Nav2 container empty (relaunched 8 s after stop, no nodes loaded; the server sends Nav2 output to /dev/null).
  - Bag 2 (`doorway_20261122_015832`): the CBF passed Nav2's commands unchanged; the **global planner** lost the path ~0.5 m from the door (`GridBased: no valid path found`, then a spin recovery). The saved map's opening is ~0.75 m (5 cm wall cells), and inscribed inflation at 0.30 m left a one-cell corridor that a live scan hit closed.
  - Changes: global costmap resolution 0.05 -> 0.025; footprint half-width 0.30 -> 0.293 (measured wheel faces) in both costmaps and the CBF default; CBF margin 0.10 -> 0.05 (`X3_CBF_MARGIN_M` still overrides).
- **Results:**
  - Bag 3 (`doorway_20261122_020804`): goal **SUCCEEDED**, robot through the centre of the doorway without touching it (user-observed), ended 0.2 m from the goal.
  - Still slow: through the doorway the CBF cut Nav2's 0.26 m/s to 0.02–0.05 m/s for ~15 s, with three "CBF infeasible" stops (nearest obstacle 0.54 m), and the planner still logged 6 "no valid path" replans. Possible next steps: a larger CBF gamma near static walls, or widening the map's doorway cells.
  - The pre-existing failure `test_cbf_tof_safety::test_pose_follows_fused_odom_not_raw` is unrelated (fails without this change too).
## 2026-11-22 — Nav map robot marker: map frame and correct rotation
- **Problem:** in the GUI, dragging a heading for Set Initial Pose seemed to do nothing, and the blue robot arrow moved wrongly compared with the real robot.
- **Cause:** the arrow was the raw `/odom` pose drawn as if it were the map frame, so an initial pose or any AMCL correction never moved it; and it was rotated by `theta + π`, which mirrors turns (left turn showed as a right turn). The lidar dots on the map used the same hand-tuned signs. The pose and heading WERE being sent to AMCL.
- **Work done** (bbce983, x3_server restarted):
  - Server: `robot_pose_map` = odom pose through AMCL's map<-odom (slam_toolbox `/pose` while SLAM runs, stored separately so the ToF exempt zones never use a SLAM frame); `laser_yaw` = laser_link's yaw from /tf_static (-3.057 rad, the ~175° mount); `/amcl_pose` subscribed unconditionally (it only was when ToF exempt zones were configured).
  - GUI: arrow and scan dots rotated with standard CCW math through the same world->screen transform as everything else; with no map pose the arrow is drawn hollow (odom, not map-corrected). Follow uses the map pose.
- **Results:** server fields verified (`laser_yaw` -3.057 matches `tf2_echo`, map pose None until AMCL publishes). **Not yet tested live:** set an initial pose with a drag and drive/rotate the robot to confirm the arrow and dots track it.

## 2026-11-22 — Seeded candidates, couch run (home)

**Problem:** the lidar extractor almost never produces a candidate for someone seated on the couch, so the tracker loses them.

**Potential solution:** camera-seeded candidates (shadow mode, f2b21a9): search for a cluster along the C3 line of sight.

**Work done:** recorded `seeded-couch-r1` (90 s, seated, leaning/shifting) and `seeded-couch-empty-r1` (45 s, nobody). Analysed `lidar_persons.ndjson`.

**Results:**
- Seated run: C3 saw the person in 838 of 892 frames. A lidar track existed in only **32 (4%)**. A seeded candidate filled the gap in **692 more**, so coverage would go **4% → 86%**. In 114 camera frames neither produced anything.
- Seeded rows sit on the camera estimate: median 0.04 m, p90 0.11 m (max 1.28 m). One row per frame.
- Empty run: C3 reported nothing, so there were 0 seeds and 0 seeded false alarms. This only shows that seeding cannot invent a person without a camera detection. It is not a test of camera false positives.
- Caveat: the seeded clusters have median z_hi 1.97 m, too tall for a seated head. The cluster probably merges with the couch back or wall. Gate on the classifier or crop z before the tracker uses these.
- Next: score seeded clusters with the classifier, then put tracker use behind a flag. Lab desk, walk and empty runs come later.

## 2026-11-22 — Camera-seeded lidar candidates live in shadow mode
- **Problem:** a seated person merges with the desk or chair in the lidar clusters, so no candidate exists and the tracker loses them. The camera still sees them.
- **Potential solution:** `seeded_candidates()` (merged from `t3/camera-seeded-lidar-person`; offline: home seated 7–30% → ~99%) cuts a body out of the cloud along each camera person's line of sight.
- **Work done:**
  - x3_server publishes C3 camera people (odom XY + velocity) on `/c3/person_seeds` at 10 Hz.
  - `livox_scan_node` predicts the seeds to the lidar frame end (it ignores seeds more than 0.3 s old). It then runs `seeded_candidates` and publishes the extra rows on `/livox/person_candidates_seeded`. `/livox/person_candidates` is unchanged.
  - The server records the seeded rows in `lidar_persons.ndjson` (`seeded`) and logs `[seeded-shadow]` counts every 60 s. **The tracker does not use them yet.**
  - `record_person_lidar.sh` now bags both new topics. Test extended (seeds used when fresh, ignored when stale).
  - Deployed (ce33a58).
- **Results:**
  - Live on the robot: seeds at ~9 Hz while someone is in view. In the first minute, 220 frames had a seeded body, and none of them was within 0.5 m of an existing lidar track.
  - Clocks line up: seed stamps are within 0.07 s of the lidar frame stamps.
  - The 4 failing test files (cbf_tof_safety, oak_depthai_v3, robot_constants, velocity_timing) also fail without this change.
  - **Next:** recorded shadow sessions (seated at a desk, walking, empty room), then classifier scores on seeded rows, then enable.

## 2026-11-22 — Navigation map: zoom, pan, follow, goal heading, scale bar
- **Problem:** the nav map was fixed at whole-map scale, goals always had heading 0, and nothing showed distance.
- **Potential solution:** a view layer (zoom + pan) on top of the existing fitted map transforms, and RViz-style press-drag for heading.
- **Work done** (975164a, 862acff; GUI only, no restart):
  - `canvasPxToWorld` / `worldToCanvasPx` now include the view, so every overlay and click follows zoom and pan; the map background is drawn under the view transform, overlays stay a fixed on-screen size.
  - Wheel or pinch zoom about the cursor (1-30x), right/middle-drag or two-finger pan, + / − / Fit / Follow buttons on the map (Follow keeps the robot centred and turns off on manual pan).
  - Left press-drag sends the goal, or the initial pose in Set Initial Pose mode, with the drag direction as `theta` (map frame); a plain click still sends 0. A heading arrow is drawn for the goal and while dragging. Scale bar bottom-left (0.1-50 m).
  - Fixed: `setPointerCapture` could throw and abort the press handler.
- **Results:** checked in a browser against the robot with RealLab3 loaded (sends stubbed, no goal reached Nav2): drag right -> 0°, drag up -> 90°, click -> 0°; the world point under the cursor stays fixed while zooming; screen/world round trip exact; right-drag pan and zoom compose correctly. **Not tested:** a real touch device (pinch) and a dragged goal executed by Nav2.

## 2026-11-22 — Navigation tab: goal progress, failure reasons, Navigate / Map modes
- **Problem:** the Nav tab showed only distance remaining, so a stopped robot gave no reason; Nav2, SLAM and Auto-Explore controls sat in one long column.
- **Potential solution:** surface the NavigateToPose feedback, infer the abort reason from Nav2's own log lines (Humble's result has no error code), and split the tab into two modes.
- **Work done** (0741a11, 5312e47, d849766):
  - `nav2_client.py`: elapsed time, ETA and recovery count from the feedback; a `/rosout` listener collects WARN/ERROR lines from Nav2 nodes during a goal and `classify_nav_failure()` maps them to a reason (no progress, no path, goal or robot in an obstacle, controller failed, TF). Test: `tests/test_nav_failure_reason.py`.
  - Speed from `/odom` in `nav_health`; a stats row and a red failure line in the GUI.
  - Navigate / Map toggle (CSS-only hiding, every id kept), remembered; starting SLAM or launching Nav2 switches mode.
  - Fixed the Nav2 status badge, which was never coloured (`'nav-badge' + state` had no space).
- **Results:**
  - Checked in a browser against the robot at desktop and phone sizes; 889 tests pass (failures are the pre-existing ones).
  - The first deploy compared `Log.WARN` (bytes in rclpy) with an int, which raised in the rclpy spin thread and stopped all ROS callbacks for about a minute; fixed in 5312e47 and the callback is now guarded.
  - Live test 2026-11-22 (apartment): goal clicked inside a wall -> the GUI showed "Planner found no path to the goal" (server log: infeasible / no path). The "stopped making progress" case is not yet seen live.
  - Seen in the same run: the robot could not drive through the 35 in (0.89 m) doorway into a room. Likely the CBF still sized for the old chassis; left for a separate thread (resize to the x3_v2 footprint and tune).

## 2026-11-22 — Navigation tab: hide dead 3D controls, SLAM health, localization badge
- **Problem:** with the tilt mount removed, 3D Scan / 3D Mapping / Save 3D Map do nothing; nothing showed whether SLAM was mapping or whether AMCL was localized (the A/B test drove on an unconverged AMCL).
- **Work done** (ec6433c, 0335d8c):
  - `hello` carries `tilt_available` (`/dev/openrb150`); without it the tilt-only controls are hidden.
  - `readout_slow` carries `nav_health`: `/scan` rate, `/map` count and age, `/amcl_pose` covariance.
  - GUI: a SLAM line (green mapping / amber starting or idle map / red no scans or no map after 15 s) and a LOCALIZED / CONVERGING / LOST / NOT LOCALIZED badge (0.15 m and 8°, 0.5 m and 25°) shown only while Nav2 is launched.
- **Results:** deployed; fields verified over the WebSocket (`/scan` 10.3 Hz, no tilt). Two old notes were stale: the `set_amcl_map` (Load map) handler exists again, and SLAM does get `/scan` from the Livox node.

## 2026-11-22 — GUI Shut down button, and the poweroff stall in x3_server
- **Problem:** no way to power the Orin off safely from the GUI; once added, the first GUI poweroffs never finished (fan and LEDs stayed on).
- **Potential solution:** a confirmed button that runs `systemctl poweroff`; persistent logs to see where shutdown stops.
- **Work done:**
  - Header button with a confirm dialog (01edea8, dialog-order fix 690a3dd); server stops the wheels, then `sudo -n systemctl poweroff`, allowed by `deploy/sudoers/x3-poweroff` (installed on the robot).
  - Made the AGX journal persistent (`/etc/systemd/journald.conf.d/persistent.conf`, `SyncIntervalSec=1s`); it was RAM-only, so every power cut erased the evidence.
  - The logs showed x3_server ignoring SIGTERM during poweroff (camera thread still running 86 s later; a plain stop took 4 s). Added a 12 s exit watchdog with a thread dump (0dbff25), then a 3 s cancel that logs where each straggling loop is parked (f28c4fc). That named `log_push_loop`, blocked forever sending to a browser after Tailscale had stopped; its send now has a 2 s timeout (2129ac7). A websocket close timeout (89423f1) was a wrong first guess and is harmless.
- **Results:** a GUI poweroff now completes in about 5-7 s (fan and LEDs off). The final send-timeout fix is deployed but has not been through another real poweroff.

## 2026-11-21 — Surroundings 3D: 2.5 cm near-field option
- **Problem:** the Surroundings view's 5 cm voxels are coarse up close (request from the calibration thread).
- **Potential solution:** 2.5 cm cells within 3 m, 5 cm beyond, at no more than 2x the current bandwidth.
- **Work done** (e56736e): `livox_scan_node.display_voxels()` publishes the mixed cloud with a 12000-point cap; the GUI adds a "Near (2.5 cm…)" option (5 cm stays the default), the stabilising memory keeps near cells only in that mode, and the buffers were raised.
- **Results:** measured on live data: before 6000 points / 352 KB/s (always capped); uncapped mixed ~15k points (2.5x), so the cap is 12000: ~700 KB/s (2.0x), ~15 ms per frame. Deployed (colcon build + restart); `/surroundings/points` verified at 12000 points, 4.9 Hz.

## 2026-11-21 — GUI camera grid, restyle, OAK feed frame rate
- **Problem:** camera feeds scrolled and lost aspect ratio; colours were hard-coded; front/rear feeds ran at ~8 fps.
- **Work done:** a camera grid that fits the panel with offline cameras as chips and a Hide-offline toggle, a near-black restyle with one accent (#d9773f) and status colours as tokens (e514d56); OAK feed throttle fixed (574056f, c742e81).
- **Results:** profiling showed the AGX ~90% idle and the laptop browser at 165 fps; the fps cap was server-side. Feeds now 15 / 15 / 17 fps, ~1.34 MB/s total.

## 2026-11-22 — Synthetic seated people and furniture for the lidar person classifier (step 0 of the Isaac Sim plan)
- **Problem:** the lidar person classifier fails on seated people merged with desks and chairs, close range and crowds. `notes/plans/2026-11-21-isaac-sim-lidar-people.md` asks whether synthetic seated and furniture content helps before anyone spends 4 days on an Isaac Sim spike (branch `t3/synthetic-seated-furniture`, nothing on main).
- **Potential solution:** extend `synthetic_people.py` (ellipsoid bodies cast onto the real Livox rays of recorded frames) with seated poses, furniture and close-range placements, then retrain with it and score on the real runs only.
- **Work done:**
  - `synthetic_people.py`: a seated body (seat height, torso lean/slouch, thighs forward, forearms on the desk or in the lap), box raycasting (`raycast_boxes`), furniture (office/dining chair, desk with legs/panels/pedestal/monitor, couch, tables, bookcase, partition), scene samplers (`sample_furniture_scene`: desk, chair, couch, standing person beside or behind furniture; `sample_close_group`: 0.4-1.5 m). Furniture can abut real clutter, nothing may touch the sensor. 24 unit tests in `tests/test_synthetic_people.py`; the standing body is bit-identical to before.
  - `synthetic_seated_experiment.py`: builds the datasets (3 generator seeds, 13k scenes each, real backgrounds from all 36 runs) and runs leave-one-run-out. Synthetic rows are labelled from point counts (>= 0.6 of a candidate's points are one person's, the sitter's own chair counted as neutral; negatives are furniture-only or person-free copies of the same scenes), and the held-out run's own backgrounds are dropped from its fold. The existing generator never fed training (`synth.npz` has no labels), so arm b is regenerated through the same path.
  - Arms, all scored on the 36 REAL runs: a real only; b + existing synthetic; b_big (same, std A+B: 4.4k rows against c_all's 8.7k, so c_all - b_big is NOT volume-controlled); c_all (+ new content); c_new, c_pos (positives only), c_neg (negatives only), c_loose (purity 0.35), b_w25/c_w25 (weight 0.25); d synthetic only. Go/no-go rule written into the script before the first run.
- **Results** (`artifacts/lidar-person-classifier-2026-11-01/results_synth_seated_2026-11-22.txt`):
  - **No arm helps; most lose.** Overall AP, mean of the three per-seed LORO values (a's seeds are the HGB fit seed; seed 0 reproduces the 0.904 baseline, the HGB seed alone spans 0.892-0.904): a 0.897, b 0.869, b_big 0.850, c_all 0.831; down-weighted b_w25 0.880, c_w25 0.853; synthetic only 0.50. Paired run-bootstrap on the seed-averaged probabilities: b_big - a -0.036 [-0.089, +0.008] (includes zero), c_all - a -0.052 [-0.102, -0.004]. The loss sits mostly in the three crowd runs of one lab, so "hurts" is a point estimate; "does not help" is solid.
  - Seated runs (16, pooled; here and below AP of the seed-averaged probabilities): a 0.980, b_big 0.979, c_all 0.971; c_all - a -0.010 [-0.052, +0.014], c_all - b_big -0.010 [-0.052, +0.013] (inside the noise against every reference). The classifier already scores real seated candidates at recall 0.98, so there is nothing to win there. Subsets (exploratory): apartment chair/couch/desk +0.021 [-0.004, +0.052], lab desk 11-20 -0.037, furniture-r1 + goal-on-r4 -0.088 [-0.142, -0.013].
  - The loss on furniture-r1 comes from the furniture negatives (seated recall 1.00 -> 0.07 with c_all; negatives-only c_neg 0.00 on furniture-r1, positives-only c_pos 0.98). Not caused by contaminated negatives: requiring 80% furniture points instead of 30% (drops 9% of the y=0 rows) leaves c_neg at 0.05 and c_all at 0.49. Likely reason, not proven: its real sitter is a 0.71 m tall, 1139-point blob at 0.69 m (z_hi 1.0), closer to the synthetic empty chairs (height 0.94) than to the synthetic sitters (1.08, z_hi 1.27).
  - Crowd runs lose with every synthetic arm (0.727 -> 0.636 for c_all); close runs are flat (+-0.002).
  - Fidelity gap: a classifier separates synthetic from real person candidates at AUC 0.98-1.00 (random halves of the real runs: 0.42). Real standing people carry ~40% more points and wider legs than the bodies; seated medians differ by > 10% in all three range bands.
  - Extractor on synthetic seated people: found 50% desk / 66% chair / 29% couch, 76-80% without the furniture, so furniture costs 10-30 points, mostly as no candidate rather than a merge. Real numbers from earlier notes are 0.95-1.0 (apt-desk2 back view) and ~5-10% (apartment chair/couch): the generator is too pessimistic for desks and too optimistic for chair/couch, so its extractor rates cannot steer `person_candidates.py`.
  - **Go/no-go for the Isaac Sim spike: NO-GO** under the rule written beforehand (seated gain >= +0.02 with the CI above 0: got -0.010 [-0.052, +0.013]; overall AP -0.06; furniture-r1 collapses). A fidelity gap remains (AUC 0.98), so this cannot tell "synthetic content does not matter" from "content at this fidelity does not transfer"; the plan only starts the spike when content helps first, and it did not. What the result does remove is the classifier-side case: real seated candidates already score about 0.98 AP and recall with real data only, so there is little for any synthetic data to add there. The extractor-side case is untested: this generator's extractor rates are uncalibrated in both directions (too pessimistic for desks, too optimistic for chair/couch), and the extractor-side seated gap is already being closed by the camera-seeded candidates on `t3/camera-seeded-lidar-person` (home seated 7-30% -> ~99%). A spike would first have to pass the plan's fidelity gate (medians within 10% at 1-4 m), which this ellipsoid generator fails.
  - Limits: 0.31 m sensor height used for the 0.302 m v2 chassis, no robot ego-motion, solid-box furniture, three generator seeds, subset numbers rest on few runs.

## 2026-11-22 — Simulation for lidar person training: step 0 started, REAL-1 prepared for Isaac Sim
- **Problem:** the person classifier lacks seated, furniture-adjacent, close-range and crowd examples.
- **Potential solution:**
  - Step 0: add seated people and furniture to the existing synthetic generator, rendered on real Livox rays, and retrain.
  - Step 1, if step 0 shows realism is the gap: an Isaac Sim 5.1 trial on REAL-1.
- **Work done:**
  - Step 0 delegated to a background agent on branch `t3/synthetic-seated-furniture`.
  - On REAL-1, with shash's verbal permission: logged out his session (his 6-day-old `thesis_robot` detection nodes were holding ~3 GB of the GPU) and added `kinova` to the docker group.
  - Started pulling `nvcr.io/nvidia/isaac-sim:5.1.0`; no NGC login was needed.
- **Results:** pending. On REAL-1 the GPU is free (RTX 4080 16 GB, driver 580.178) and Docker has the NVIDIA runtime.

## 2026-11-21 — Left camera ↔ Livox extrinsic
- **Problem:** the left camera needs its pose relative to the Livox before lidar points can be matched to people it sees.
- **Potential solution:** the same 8-station capture as the other cameras, solved with koide3.
- **Work done:**
  - Captured 8 stations (`calib_capture.py --cameras left`, rotated 180°) and rectified them with the new intrinsics; door edges come out straight.
  - The first seed had the image upside down. With the corrected seed, koide3 still returned NaN on the first step every time, even though the data looked clean.
  - Instead, solved with a Python edge alignment: lidar depth-edge points against camera edges, starting from a seed mirrored from the right camera.
- **Results:**
  - T_lidar_camera = (0.005, 0.057, -0.077) m, about 0.7° from the seed.
  - Mean edge distance 8.7 → 7.1 px; the table legs, wagon and door frame overlay correctly.
  - Saved as `left` in `config/camera_lidar_extrinsics_2026-11-20.json`.
  - Added to the URDF the same evening (92343aa, deployed): `left_cam_link` and `left_cam_optical_frame` hang off `livox_frame`, and the live TF matches the JSON. A 2.1 mm M12 lens (~110° HFOV, matching the right camera) is planned, after which the lens calibration and extrinsic will be redone.
  - The koide3 NaN cause is still open.

## 2026-11-21 — Left camera lens (fisheye) calibration
- **Problem:** the left camera needs a lens model before it can be calibrated against the lidar.
- **Work done:**
  - Captured 35 checkerboard views on the robot (9x6 board, 23.3 mm squares), rotated 180° to match the published image.
  - Refit on the laptop without views whose per-view error was > 1 px. The 9 dropped views mostly had the board partly out of frame.
  - Saved `config/left_u20cam_intrinsics.json`.
- **Results:**
  - Error went from 1.57 px on the robot's fit to **0.47 px** on 26 views.
  - fx 1119.7 px, centre (687.8, 378.2); the lens covers about 69° horizontally and 42° vertically, a much narrower lens than the right Arducam's ~112°.
  - Pinhole models fit worse (0.70 px).
  - The image corners are thinly covered (6–8 corner points in the top-left and bottom-right cells), so the edge distortion is extrapolated. The right camera reached 0.09 px, so this fit is **provisional**.
  - **Top-up (same day):** 15 more views with the board near the corners (robot fit 2.0 px). Merged refit: **0.43 px on 36 of 50 views**, fx 1119.8 / fy 1119.0, centre (687.8, 378.2). The parameters barely moved, so the fit is stable. Top-left coverage 6 → 31 points, bottom-right still thin (11). Good enough for the lidar extrinsic.

## 2026-11-21 — ROS data-plane stall: root cause found and fixed (logind RemoveIPC)
- **Problem:** every ROS topic went silent about 1–3 minutes after a restart, while the Livox kept sending (~2,300 packets/s on eno1).
- **Potential solution:** find what breaks FastDDS. Its shared-memory transport carries all on-robot traffic.
- **Work done:**
  - Saw that the `/dev/shm/fastrtps_*` files were gone during the stall, with the processes still running.
  - Reproduced it: 50 files before an SSH session ended, 0 after.
  - systemd-logind's default `RemoveIPC=yes` deletes a user's shared memory when that user's last login session closes. The services run as `jetson`, and every SSH command opens and closes a `jetson` session.
  - Fix on the robot: `loginctl enable-linger jetson` (takes effect immediately) plus `/etc/systemd/logind.conf.d/10-x3-keep-ipc.conf` with `RemoveIPC=no` (persists; copy kept in `deploy/logind/`).
- **Results:**
  - After the fix, the files survived two SSH sessions closing and the watchdog stayed quiet.
  - This explains the "~1 in 3 startups" pattern: the stall happened whenever someone's SSH session to the robot ended after the services had started.
  - Restarting the services only worked because it recreated the files.
  - The watchdog stays as a guard.

## 2026-11-21 — Data-plane stall came back after server restarts (watchdog caught it)
- **Problem:** the Surroundings view was empty on the laptop.
- **Work done:** read the watchdog log. Then restarted x3_server and, 15 s later, foxglove_bridge.
- **Results:**
  - The watchdog logged an ERROR after **both** left-camera deploy restarts (21:39:53, 21:41:56). In both, /scan and /odom delivered data for about a minute (345–485 scans), then stopped. Earlier stalls never delivered anything, so this is a new pattern.
  - After the restart, no stall in 90 s.
  - The detector works. Auto-recovery and a GUI banner are now more urgent, since a plain server restart can trigger the stall.
  - Bringup was running with `floor:=lab_carpet` in the apartment (strafe scale and ToF baseline).

## 2026-11-21 — Left camera feed in the server and GUI
- **Problem:** the new left camera (Innomaker U20CAM-9281M) was plugged in but unused, and its image is rotated 90°.
- **Potential solution:** capture it in the existing role-tagged camera feed loop as the `left` role (the GUI already understands it) and rotate each frame 90°.
- **Work done:** `_left_capture()` in `server_x3.py` (2e3da35, deployed). It opens the camera by its by-id path at 640x400, rotates the frame, and reopens the camera if it drops. `X3_LEFT_CAM=none` turns it off.
- **Results:**
  - On the robot websocket, front, left, right and rear all arrive at about 14–17 fps. The first rotation (clockwise) was upside down; the user then remounted the camera landscape, and raw frames are upside down, so they are now rotated 180°.
  - **Next:** the user adjusts the lens. Then intrinsics, a Livox extrinsic, and the URDF.

## 2026-11-21 — Clip fix deployed; left camera (Innomaker U20CAM-9281M) connected
- **Problem:** the C3 box-clip fix was merged but not on the robot. A new left camera was plugged in.
- **Work done:** robot pulled to 95b2c49 and x3_server restarted. Identified the new camera and grabbed test frames.
- **Results:**
  - x3_server is up with the C3 tracker running, and the stall watchdog logged no error.
  - The left camera shows up as `0c45:636d` "USB 2.0 Camera", serial SN0001, at `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0` (video0). It does not collide with the right Arducam (`0c45:6366`, UC599), which the server finds by its "Arducam" by-id name.
  - It gives mono 1280x800 and 640x400 at 10 fps (V4L2 default, MJPG not yet tried). The image is rotated 90° (the outlet appears sideways), so the camera is mounted sideways.
  - **Not done yet:** the camera isn't used by the server or GUI, has no intrinsics or Livox extrinsic, and is not in the URDF. A live re-check of the seated-range fix also hasn't happened.

## 2026-11-21 — Livox position in the person dataset tool was the old chassis'
- **Problem:** `lidar_person_dataset.py` placed the Livox at x = +0.0432 m (old chassis). On x3_v2 it is at x = -0.0025 m, so every 11-20 lidar range was ~4.6 cm off. The lidar height also disagreed between files: the URDF says 0.2731 m, the perception tools 0.31 m.
- **Potential solution:** pick the Livox position per chassis from the run date (rebuild 2026-11-13). Measure the height from floor returns.
- **Work done:** `livox_xyz_for()`: old chassis (0.0432, 0, 0.31), x3_v2 (-0.0025, 0.0003, 0.302). Test added. The floor was measured in the recorded bags.
- **Results:**
  - Floor sits 0.302 m below the Livox on x3_v2 (11-20 lab) and 0.305 m on the old chassis (11-09). The 0.31 m used by the extractor and features is within 1 cm.
  - The URDF's 0.2731 m is ~3 cm low; probably the housing, not the measurement origin. **The URDF is not changed yet.**
  - 11-20 labelling barely changes, because the camera range error dominates.

## 2026-11-21 — Camera (C3) reads seated people ~0.8 m too far
- **Problem:** at the lab desk, the camera put the seated user 0.74–0.93 m behind where the lidar sees them. Lidar candidates then failed to match the camera person, so the desk runs gave almost no training labels.
- **Potential solutions:** focal-length (fx) scale error; constant offset; depth read off the person.
- **Work done:**
  - Offline comparison of camera range vs lidar front-surface range across all runs (branch `t3/analyze-camera-range-error`, ade5ee8).
  - C3 box-clip fix plus an optional nearest-surface depth mode (`t3/c3-depth-roi-fix`, f2e4608, **merged into main**).
- **Results:**
  - Cause: every 11-20 person box starts 114–240 px above the image, so the torso depth window read the ceiling and partition behind the user.
  - Clipping the box to the image halves the error (+0.87 → +0.41 m).
  - fx scale and constant offset were **ruled out**.
  - Nearest-surface mode did not help (left off).
  - Walkers still read +0.2 m too far beyond 1.5 m (cause open).
  - C3 discards depth beyond 4.0 m.
  - **Next:** read depth only from the person's segmentation outline, then fuse the lidar range.
  - The clip fix is **not deployed yet** (robot was off).

## 2026-11-21 — Isaac Sim / Lab for lidar person training (feasibility)
- **Problem:** the classifier lacks seated, crowd and close-range examples. Could simulation supply labelled data?
- **Potential solution:** Isaac Sim with a Livox Mid-360 model and animated or seated people.
- **Work done:** feasibility study (`t3/isaac-sim-lidar-study`, 26d8647): `notes/plans/2026-11-21-isaac-sim-lidar-people.md` and `scripts/isaac_sim_preflight.sh`.
- **Results:**
  - The laptop (RTX 4060, 8 GB) is below the 16 GB minimum. The AGX is unsupported. REAL-1 (RTX 4080 16 GB) is OK and now free to use.
  - There is no official Livox model, only an unverified community profile.
  - Plan: step 0 first (seated people and furniture in `synthetic_people.py`, laptop, ~3 days). Then an Isaac Sim 5.1 trial on REAL-1 only if step 0 shows realism is the gap.

## 2026-11-21 — Camera-seeded lidar person extractor (offline)
- **Problem:** the lidar extractor merges a seated person with the desk or chair, so no candidate is produced.
- **Potential solution:** search the lidar along the camera person's line of sight and cut out a body-sized blob.
- **Work done:** `seeded_candidates()` in `person_candidates.py`, off by default (`t3/camera-seeded-lidar-person`, c0af8e4 and b4fb458), plus an eval over 27 runs.
- **Results:**
  - Apartment seated people found: 7–10% → 99–100% of frames.
  - Standing people found: 82% → 92%; a group run 75% → 99%.
  - Cost: +0.1–0.3 candidates per frame, +0.4 ms per frame.
  - Lab desk unchanged, because of the camera range error above.
  - A camera false positive in apt-desk2-empty-drive-r1 was really the user (confirmed), so false alarms are ~0.007 per frame.
  - **Next:** shadow-mode live test after the range fix.

## 2026-11-21 — Person classifier: reflectivity features and 11-20 lab runs
- **Problem:** does Livox reflectivity help the classifier, and do the lab runs add seated data?
- **Potential solution:** six reflectivity statistics per candidate; retrain on 36 runs.
- **Work done:** dataset and classifier changes (fc5c376); `near_camera` label overrides for parked runs whose odom heading had drifted.
- **Results:**
  - Leave-one-run-out AP: base 0.904; with reflectivity 0.871 (worse, so it stays off).
  - The lab desk runs added few seated rows (extractor gap and camera range).

## 2026-11-21 — Surroundings 3D view goes white (ROS data-plane stall)
- **Problem:** after a boot or restart, every ROS topic sometimes delivers nothing (/livox/lidar, /scan, /odom), while the Livox answers ping and its driver runs. The GUI silently falls back to "3D unavailable/stale".
- **Potential solution:** detect it and alert; later, restart the bringup and foxglove_bridge automatically.
- **Work done:** `src/ros_data_watchdog.py` plus a 1 Hz loop in `server_x3.py` (a05a48c, deployed). It logs an ERROR and sends `ros_data` in the readout.
- **Results:**
  - Restarting x3_server and then foxglove_bridge fixed tonight's stall.
  - Detection only; auto-recovery and a GUI banner are still to do.
  - The 2.5 cm near-field cloud option was handed to the GUI thread.

## 2026-11-21 — Front OAK calibration into the URDF; gyro check in the apartment
- **Problem:** the URDF front OAK pose came from CAD and the camera's IMU, not the lidar calibration.
- **Work done:** `oak_center_joint` now hangs off `livox_frame` with the lab-solved pose (3bac3ec, deployed). A test pins it to the calibration JSON.
- **Results:**
  - The live TF matches the calibration.
  - Parked heading drift on the apartment wood floor: −0.012° in 59 s (the gyro bias fix holds).

## 2026-11-20 — Lab session (UCR)
- **Problems:** the front camera-lidar extrinsic was provisional. Parked heading drifted (AMCL wall fit 67% → 21% in 2 min). The robot moved between runs.
- **Work done:**
  - Front OAK re-calibrated (10 stations, koide3).
  - /imu/data yaw bias tracked and subtracted in `livox_imu_relay` (d995fb8).
  - Person recordings: empty, walk, and seated (desk and open floor).
  - Person spatial prior (`person_spatial_prior.py --parked`).
- **Results:**
  - Front extrinsic corrected by 1.4° yaw and 1.8 cm.
  - Parked heading drift 0.17°/s → 0.0° in 59 s.
  - Robot moves explained: a CBF retreat from the walking user, and the user's GUI drive.
  - Seated at a desk: the camera sees the user on 85–88% of frames, a lidar track exists on only 23–27%.
