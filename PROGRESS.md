# Progress log

Newest entries at the top. Each entry: **Problem**, **Potential solution**, **Work done**, **Results**.
Dates are when the work happened. Commits are on `main` unless a branch is named.

---

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
