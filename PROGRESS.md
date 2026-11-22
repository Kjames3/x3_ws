# Progress log

Newest entries at the top. Each entry: **Problem**, **Potential solution**, **Work done**, **Results**.
Dates are when the work happened. Commits are on `main` unless a branch is named.

---

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
