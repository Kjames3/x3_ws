# EE 244 Final Project - Progress & Matrix

This document tracks the implementation progress, technical challenges encountered, and the reasoning behind each major change in the project. It also serves as the master tracking matrix for weekly deliverables.

## Project Tracking Matrix

| Week | Phase | Goal / Deliverable | Status |
| :--- | :--- | :--- | :--- |
| **Weeks 1-2** | **Data Preparation** | Preprocess THÖR-MAGNI dataset. Extract synchronized triplets at 10Hz. | Completed 🟢 |
| **Week 3** | **Initial Training** | Train and validate on THÖR-MAGNI. Hold out 20% sequences. Establish MAE baseline against Kalman filter. | Completed 🟢 |
| **Week 4** | **Domain Adaptation** | Collect 30-min domain adaptation set in deployment environment. Process rosbags offline into X/y windows matching THÖR-MAGNI format. | Completed 🟢 |
| **Weeks 5-6** | **Fine-Tuning** | Fine-tune model on collected data (learning rate ≤ 1e-4, freezing early layers) to bridge sensor gap. | Completed 🟢 |
| **Week 7** | **Demo Prep** | Focus on A/B comparison (MPPI with/without velocity estimates). Visualize in RViz2 with velocity arrows. | Completed 🟢 |

---

## Progress Log

### Week 1 - Data Preparation

### [2026-04-19 09:00 AM] Initial Setup & Dataset Exploration
**Scripts Renamed/Created:** 
- `01_fetch_zenodo_dataset.py` (formerly `download_dataset.py`) created to handle fetching the main dataset (THÖR-MAGNI) via the Zenodo API.
- `00_verify_zenodo_size.py` (formerly `download_test.py`) created to test the Zenodo API endpoint and determine the total size of the dataset.

**Reasoning:** 
We needed to verify that the dataset could be automatically downloaded to the robot/workstation, but because it contains gigabytes of data, a structural check was essential to understand the memory footprint first before downloading the 21GB payload.

### [2026-04-19 02:00 PM] Dataset Downloaded & Initial Extraction
**Scripts Renamed/Created:** 
- Dataset fully downloaded to the local workspace.
- `02_clean_and_extract_trajectories.py` (formerly `extract_data.py`) written to iteratively parse the raw `CSV_Scenarios` files and extract position coordinates for both the robot and pedestrians.

**Reasoning:** 
We needed a pipeline to translate the massive THÖR-MAGNI dataset, containing high-frequency (100Hz) motion capture points, into a structured and downsampled (10Hz) set of trajectory DataFrames that a velocity regression model could consume.

### [2026-04-19 08:30 PM] Data Quality Sanity Check & Velocity Spikes Found
**Scripts Renamed/Created:** 
- `00_inspect_csv_headers.py` (formerly `view_dataset_properties.py`)
- `00_inspect_zip_contents.py` (formerly `test_dataset_size.py`)
- `04_plot_trajectory_stats.py` (formerly `test_csv_file.py`) 

**Reasoning:** 
Sanity checks are crucial when working with raw sensor or motion-capture data. When analyzing the extracted features using our statistical plotting scripts, the output revealed impossible human speeds—specifically, a minimum velocity of -208 m/s and a maximum velocity of 215 m/s. This indicated that the raw data contained significant noise, occlusion gaps, or missing frames that resulted in violent geographical "jumps" when taking the position derivative.

### [2026-04-19 10:30 PM] Dataset Sanitization & Noise Filtering
**Scripts Modified:** 
- `02_clean_and_extract_trajectories.py` refactored entirely to implement the `clean_and_compute_velocity()` function.
- Added thresholds for physical plausibility (`MAX_SPEED_MS = 3.0` and `MAX_POSITION_JUMP_M = 0.5`).

**Reasoning:** 
To train an accurate velocity estimation network, the ground-truth velocity labels must be entirely noise-free and physically plausible. By monitoring the distance vector between subsequent timestamps, the script now catches frames where the motion capture temporarily loses track of a pedestrian and then regains them. It masks out those unnatural position jumps (and excessive gradient derivatives) with NaNs, effectively eliminating the ±200 m/s noise spikes from our final processed dataset.

### [2026-04-19 11:30 PM] Feature Windowing for ML Input
**Scripts Renamed/Created:** 
- `03_build_training_windows.py` (formerly `build_windows.py`) created.

**Reasoning:** 
With clean data extracted, the MLP regression network requires sequences of past data to predict the final velocity. We implemented a sliding window approach with parameters `T=10` (1 second history at 10Hz) and `stride=1` frame step. This script successfully partitions our cleaned pedestrian trajectories into structured `X` (features matrix, shape `T * 4`) and `y` (target `[vx, vy]`) matrices for our models, rigorously separated into Train, Validation, and Test sequences without cross-contamination.

## Week 1 Goal Complete (4 days Early)

## Beginning Week 3 - Model Training

### [2026-04-23] Initial Model Training, Evaluation, and Plotting
**Scripts Modified/Created:**
- `training/model.py`: Implemented `VelocityMLP`, a lightweight neural network (3 hidden layers: 256, 128, 64) mapping 40-dimensional temporal history (10 frames) to `[vx, vy]` outputs.
- `training/dataset.py`: Implemented PyTorch `Dataset` with `StandardScaler` to handle distribution normalizations.
- `training/train.py`: Set up the robust training loop using Huber Loss (resilient to outliers) and `ReduceLROnPlateau`. Added absolute paths dynamically derived from `__file__` to ensure the script runs reliably from any CWD. Handled package dependency issues (`scikit-learn` missing, `scipy`/`numpy` version incompatibilities).
- `training/evaluate.py`: Implemented evaluation to directly compare `VelocityMLP` outputs to a standard constant-velocity Kalman baseline on the unseen testing set. Included a `weights_only=False` fix for modern PyTorch deserialization compatibility.
- `training_plot.py`: Plotted validation and training curves via `matplotlib`.

**Results & Reasoning:**
The neural network successfully trained on the dataset, achieving early stopping optimality around Epoch 35. When evaluated on the test set, the model showed massive improvements over the constant-velocity Kalman baseline:
- **MAE Speed**: 0.2160 m/s (Ours) vs 0.6432 m/s (Kalman Baseline)
- **RMSE**: 0.3054 m/s (Ours) vs 0.8184 m/s (Kalman Baseline)
The MLP provided a ~66% improvement across all metrics. The plotted history successfully reflects this outperformance and the smooth convergence of the training run.

## Week 3 Goal Complete (9 days Early)

---

## Week 4 — Domain Adaptation

### [2026-05-19] Rosbag Infrastructure & Processing Pipeline Setup
**Scripts/Files Created:**
- `robot/record_bag.sh`: Shell script to run on the Jetson. Records `/camera/depth/image_raw`, `/scan`, `/odom` to a timestamped compressed `.db3` bag during teleoperated classroom sessions.
- `preprocessing/05_process_rosbag.py`: Offline bag processor. Reads `.db3` bags using pure `sqlite3` (no `rosbag2_py` required on laptop), detects person candidates via depth blob detection (0.5–4m range, connected components) + LiDAR DBSCAN clustering (eps=0.25m), tracks them with nearest-neighbor assignment, and outputs `X_adapt.npy (N, 40)` / `y_adapt.npy (N, 2)` in the **exact same format** as the THÖR-MAGNI windows pipeline.
- `training/finetune.py`: Fine-tuning script. Loads the best base checkpoint, freezes first 2 of 3 hidden blocks, uses 20% THÖR-MAGNI replay (mixed dataset) to prevent catastrophic forgetting, trains with lr ≤ 1e-4 using Huber loss and `ReduceLROnPlateau`.
- `robot/README.md`: End-to-end deployment guide (Jetson bringup → bag recording → scp → offline processing → fine-tuning).

**Files copied from `x3_ws` into `robot/`:**
- `drivers_x3.py`, `Rosmaster_Lib.py`, `nav2_client.py` (hardware abstraction)
- `params/nav2_params_x3.yaml`, `params/ydlidar_x3.yaml`, `params/ekf_x3.yaml` (nav config)
- `launch/x3_bringup.launch.py`, `x3_slam.launch.py`, `x3_nav2.launch.py` (ROS2 launch files)

**Reasoning:**
The domain adaptation approach shifted from live Kalman pseudo-labeling to a cleaner offline pipeline: record raw sensor streams → post-process on laptop. This avoids dependency on Jetson-side Python ML libraries during collection and lets us iterate on the feature extraction algorithm without re-collecting data. The `05_process_rosbag.py` script uses pure `sqlite3` to read `.db3` files, so it runs on the laptop without a full ROS2 install — only `numpy`, `scipy`, `opencv`, and `scikit-learn` are needed.

**Note:** The nav stack uses MPPI controller (not DWA as in the original proposal). The Week 7 A/B demo will compare MPPI with/without velocity estimates injected into the costmap.

**Status:** Completed 🟢. Classroom data has been collected, preprocessed, and cleaned.

---

## Weeks 5-6 — Fine-Tuning & Evaluation

### [2026-05-30] Cleaned Rosbags, Stage-Unfrozen Fine-Tuning, and Three-Way Evaluation

**Scripts Modified/Created:**
- `preprocessing/09_cleaning_rosbags.py`: Cleaned the collected domain adaptation rosbags dataset to filter out outliers (speed > 3.0 m/s) and balance stationary vs moving frames (targeting 40% stationary frames). Corrected directory paths to work seamlessly from any CWD.
- `training/finetune.py`: Fixed deserialization `UnpicklingError` in PyTorch 2.6+ by specifying `weights_only=False` for local config objects, and resolved `ReduceLROnPlateau` `TypeError` by removing deprecated/removed `verbose` argument. Fine-tuned the pre-trained model on 41,781 domain adaptation samples with stage unfreezing (first layer frozen for 10 epochs, then unfreezing all layers at LR = 5e-5). Achieved an optimal MAE Speed of **0.1699 m/s** (epoch 27). Exported fine-tuned model to TorchScript for Jetson deployment.
- `training/evaluate.py`: Extensively modified the evaluation pipeline to support side-by-side three-way comparison (Kalman baseline, original Base MLP, and Fine-Tuned MLP) on both the original THÖR-MAGNI test set and the Yahboom X3 Domain Adaptation validation split. Added robust mtime-based checkpoint loading, command-line arguments (`--domain`), and percentage improvement calculations.

**Results & Reasoning:**
When evaluated on the target domain (**Yahboom X3 Domain Adaptation Val Split**), the fine-tuned model achieved stellar results:
- **Kalman Baseline MAE Speed**: 0.1834 m/s
- **Base MLP Model MAE Speed**: 0.1701 m/s
- **Fine-Tuned MLP Model MAE Speed**: 0.1577 m/s (an improvement of **+14.0%** over Kalman and **+7.3%** over Base MLP)
- **vx MAE**: Improved to **0.0931 m/s** (a **+25.7%** improvement over Base MLP's 0.1253 m/s)

This confirms that the stage-unfrozen fine-tuning successfully bridged the domain/sensor gap on the Yahboom X3 robot platform.

**Status:** Completed 🟢. Ready to copy the final `velocity_mlp_finetuned.torchscript` to the Jetson robot for the Week 7 A/B controller demo!

---

### Week 7 - Demo Prep & Core Infrastructure Enhancements

### [2026-06-05] Implementation of Tier 1 High-ROI Enhancements
**Scripts Modified:**
- `server_x3.py` (ROS2Bridge & WebSocket Server)
- `velocity_estimator.py` (Inference & Centroid Extraction)
- `ab_comparison_test.py` (A/B Run Logger)

**Changes & Technical Rationales:**

1. **Depth-Aware 3D Centroid Reconstruction (Idea 19)**
   * **Change:** Exposed the raw, un-normalized floating-point depth array (in meters) from `ROS2Bridge._depth_cb` via a new `get_raw_depth_frame()` method. Updated `VelocityEstimator` to draw contours on the colorized depth frame, mask those contour areas on the raw depth array, extract the median depth $Z$ in meters, and compute metric coordinates using the pinhole model: `x_m = (cx - w / 2.0) * Z / fx`.
   * **Rationale:** In the original estimator implementation, coordinates were scaled as if $Z$ was a constant $1.0\text{ m}$. This caused physical lateral distances and displacements to be underestimated by a factor of $Z$ (e.g., a $300\%$ coordinate scale compression for obstacles at $3.0\text{ m}$). Since the MLP models were trained on absolute metric displacements from THÖR-MAGNI and raw offline depth clusters, this correction aligns the live input feature distributions with the trained network weights, restoring velocity estimation accuracy.

2. **Pedestrian Clearance Distance Logging (Idea 22)**
   * **Change:** Updated `ab_comparison_test.py`'s `RunLogger` and `_maybe_log()` methods to read the `(x, z)` relative coordinates of all tracked obstacles, calculate their Euclidean distance from the robot center ($d = \sqrt{x^2 + z^2}$), find the minimum clearance $d_{min}$, and write it to a new `"min_obstacle_distance"` column in the CSV log files at $10\text{ Hz}$.
   * **Rationale:** The project proposal explicitly lists "minimum clearance distance" as one of the three core quantitative navigation metrics to compare the reactive vs. predictive planning modes. Without logging the distance coordinates alongside the robot's pose, the team would have had no way to analyze safety metrics post-run.

3. **ROS2 Topic & RViz2 Marker Publishing of Pedestrian Poses (Ideas 2 & 9)**
   * **Change:** Initialized `/pedestrian_poses` (`geometry_msgs/PoseArray`) and `/pedestrian_markers` (`visualization_msgs/MarkerArray`) publishers inside `ROS2Bridge`. Implemented coordinate frame transformations from the camera optical frame ($X$=right, $Z$=forward) to the standard REP-103 robot base frame (`base_link`: $X$=forward, $Y$=left) so that Robot $X = Z_{camera}$ and Robot $Y = -X_{camera}$. Published oriented arrows (color-coded by speed) and text tags showing IDs and velocities.
   * **Rationale:** This satisfies two key objectives in the EE244 proposal: providing a live ROS2 stream of estimated obstacle velocities to feed Nav2's MPPI dynamic costmap layer, and providing oriented velocity arrows and text markers for live RViz2 diagnostic projection.

4. **Lifecycle Safety & Subprocess Orphan Prevention (Idea 17)**
   * **Change:** Added robust cleanup handlers for `_ab_test_proc` in the global `cleanup()` function of `server_x3.py`, executing clean `.terminate()` and fallback `.kill()` routines to cleanly reap any running A/B test process when the server shuts down or restarts.
   * **Rationale:** Preventing orphaned background python tasks from running after a server exit is critical on a physical robot. If the websocket server is restarted, any running test script would otherwise continue publishing velocity commands to `/cmd_vel` in the background, creating a physical safety hazard.

**Status:** Completed 🟢. All Tier 1 changes are successfully coded, structured, and integrated into the local workspace.

### [2026-06-05] Boot Synchronization & Bug Fixes
**Scripts Modified/Created:**
- `velocity_estimator.py` (Fixed centroid tuple-to-triplet conversion)
- `x3_server.service` (Updated network target dependencies and IP resolution loop)
- `orbbec_depth.service` (Updated network target dependencies and IP resolution loop)
- `scratch/deploy_code.py` (Deployment automation script)

**Changes & Technical Rationales:**

1. **Centroid Tuples-to-Triplets Conversion (Inference Crash Fix)**
   * **Change:** Modified the `_extract_depth_centroids()` function in `velocity_estimator.py` to return the full `(x_m, y_m, Z)` coordinate triplet (where $Z$ is the raw depth in meters) instead of the old `(x_m, y_m)` coordinate tuple.
   * **Rationale:** In the previous update, `ObstacleTracker` was modified to propagate 3D coordinates. However, `_extract_depth_centroids()` was left returning 2-element tuples. This resulted in `ValueError: not enough values to unpack (expected 3, got 2)` inside `ObstacleTracker.update()`, crashing the estimation thread on boot and preventing manual gamepad control and autonomous scripts from running. Correcting the return value resolves the crash.

2. **Systemd Boot Synchronization & Real IP Address Resolution (Race Condition Fix)**
   * **Change:** Updated `x3_server.service` and `orbbec_depth.service` to depend on `network-online.target` (instead of `network.target`) to guarantee the robot has finished establishing its Wi-Fi connection. Added a robust bash loop in `ExecStart` that queries `hostname -I` for up to 30 seconds to wait for a valid, non-loopback IP address before exporting the `ROS_DISCOVERY_SERVER` variable.
   * **Rationale:** On cold boots, the services were starting before the Wi-Fi card had finished connecting. Because `hostname -I` was empty, `x3_server` defaulted its discovery server environment to `127.0.0.1:11811`. Meanwhile, `orbbec_depth` started slightly later, resolved the real IP `10.13.247.117:11811`, and registered on a different locator. This network partition blinded the websocket server and broke the `/cmd_vel` control pathway. Forcing the services to wait for a valid network IP guarantees they always start on the same discovery locator.

**Status:** Completed 🟢. Both local files and robot service configurations have been fully deployed and tested on the physical robot.

### [2026-06-05] Implementation of Tier 2 High-ROI Enhancements
**Scripts Modified:**
- `velocity_estimator.py` (Meters-based raw depth processing)
- `point_to_point_test.py` (PD angular control loop)
- `ab_comparison_test.py` (PD angular control loop & Proximity/TTC speed scaling)

**Changes & Technical Rationales:**

1. **Direct Meters-Based Depth Processing (Idea 21)**
   * **Change:** Refactored `_extract_depth_centroids()` in `velocity_estimator.py` to perform thresholding directly on the raw depth array values (`0.5m <= depth <= 4.0m`) rather than converting the depth image to a normalized BGR color map first and thresholding the grayscale representation.
   * **Rationale:** Grayscale thresholding on color-mapped BGR frames is highly susceptible to background scaling fluctuations (e.g. when the robot rotates and the min-max normalization bounds shift). Processing the raw physical depth values directly makes obstacle extraction completely invariant to background shifts, prevents tracking dropouts, and reduces CPU utilization by skipping the BGR and grayscale color-conversion pipelines.

2. **PD-based Closed-Loop Heading Control (Idea 3)**
   * **Change:** Introduced derivative ($D$) tracking and a Proportional-Derivative (PD) control loop (`KD_ROT = 0.1`) in both `point_to_point_test.py` and `ab_comparison_test.py`'s turnaround states.
   * **Rationale:** Proportional-only heading control exhibits momentum overshoot and oscillations when settling on $180^\circ$ turnarounds. The derivative term acts as an active dampener, slowing down the angular rotation speed dynamically as the heading error drops to zero, ensuring clean, overshoot-free orientation locking.

3. **Dynamic Proximity & Time-to-Collision Speed Scaling (Idea 13)**
   * **Change:** Implemented a real-time relative-coordinate proximity and 2D Time-to-Collision (TTC) speed scaling calculation in `ab_comparison_test.py` using the asynchronous obstacle estimation states. Throttles the robot's linear speed inside driving segments and commands a safety stop ($v = 0.0$ m/s) if a pedestrian approaches closer than $0.8$ m or has a TTC below $1.0$ second.
   * **Rationale:** Proactively slowing down linear velocity when an obstacle approaches ensures safety, makes the robot's movement look cautious and natural, and reduces the risk of collision or aggressive local planning maneuvers in narrow indoor environments.

**Status:** Completed 🟢. All Tier 2 improvements are successfully coded and verified in the local workspace.

### [2026-06-05] Holonomic Strafing Bypass & LiDAR Safety Fusion (User Enhancements)
**Scripts Modified:**
- `velocity_estimator.py` (Non-blocking toggle for estimation logic)
- `server_x3.py` (Toggled estimation mode in estimator state directly)
- `ab_comparison_test.py` (LiDAR scan subscription, holonomic bypass offset, lateral control, paused time accounting)

**Changes & Technical Rationales:**

1. **Non-blocking Estimator Lifecycle Toggling**
   * **Change:** Added `self.estimation_enabled` state to `VelocityEstimator`. Instead of terminating and recreating the thread which adds CPU overhead and socket reset delays, the websocket server toggles this state. If disabled, the estimator skips running network forward-passes and defaults velocity estimates to `0.0`.
   * **Rationale:** Optimizes system performance and eliminates lag/reconnect latency when turning the pedestrian estimator ON and OFF from the GUI.

2. **LiDAR Safety Fusion (Blind Spot Protection)**
   * **Change:** Subscribed to `/scan` inside `ab_comparison_test.py`. Implemented a safety threshold check for obstacle points right in front of the robot (`0.15m < distance < 0.75m` within $\pm 30^\circ$ cone).
   * **Rationale:** Provides secondary redundancy for camera depth tracking. If a pedestrian steps into the camera's blind spot extremely close to the front bumper, the LiDAR catches them and forces a safety stop.

3. **Holonomic Mecanum Bypass Strafing (Lateral Avoidance)**
   * **Change:** Integrated a lateral bypass controller inside `ab_comparison_test.py`'s `DRIVE_TO_B` and `DRIVE_TO_A` states. When an obstacle is detected blocking the forward path corridor, the robot pauses its forward drive and commands lateral strafing speed `twist.linear.y = vy_cmd` to shift left or right by $0.7$ meters (bypassing the obstacle).
   * **Rationale:** Fully utilizes the Yahboom X3 mecanum-wheel holonomic base. Instead of standing stuck in front of a pedestrian, the robot can slide sideways to clear the obstacle, resuming forward drive once the path is clear.

4. **Non-blocking Timeout Accounting**
   * **Change:** Paused the segment safety timer (`state_elapsed_time`) whenever the robot is in a blocked/avoidance pause state.
   * **Rationale:** Prevents the $20\text{ s}$ safety timeout from shutting down the autonomous run when the robot is waiting for or navigating around a pedestrian.

**Status:** Completed 🟢. The holonomic bypass and safety features are successfully integrated into the test pipeline.

### [2026-06-05] Tight Space Navigation, Side Wall Avoidance & Symmetric Return Paths
**Scripts Modified:**
- `ab_comparison_test.py` (Reduced lateral thresholds, static wall detection, side-wall LiDAR check, symmetric return path, absolute target headings, adaptive corridors)
- `velocity_estimator.py` (Mapped camera features `cx, cz` to robot coordinates `ry=-cx, rx=cz` to match trained MLP inputs)
- `scratch/deploy_code.py` (Updated IP address to `10.13.197.182` for robotics lab)
- `scratch/read_logs.py` (Updated IP address to `10.13.197.182` for robotics lab)

**Changes & Technical Rationales:**

1. **Tight Space Avoidance Tuning**
   * **Change:** Reduced the lateral path corridor from `0.8` m to `0.5` m, and the bypass offset from `0.7` m to `0.4` m.
   * **Rationale:** In tight indoor environments (like apartments or narrow lab corridors), the robot needs to ignore static side furniture (e.g. table legs and chairs) and reduce its lateral evasion envelope to avoid colliding with boundaries.

2. **Static Wall vs. Dynamic Pedestrian Detection**
   * **Change:** Integrated a fusion check between the camera estimates and the LiDAR scan. If the front LiDAR is blocked but the camera detects no dynamic pedestrian tracker candidate, the robot treats the obstacle as a static wall and pauses safely in place without trying to strafe/bypass.
   * **Rationale:** Prevents the robot from continuously attempting to slide sideways (strafing) when facing a static boundary, which would otherwise result in scraping or getting stuck against side walls.

3. **Side Wall Collision Avoidance (LiDAR Sector Scanning)**
   * **Change:** Programmed the robot to scan left ($60^\circ$ to $120^\circ$) and right ($-120^\circ$ to $-60^\circ$) LiDAR sectors before and during lateral strafing. If the chosen bypass direction has an obstacle closer than `0.45` m, it attempts to bypass on the opposite side. If both sides are blocked, it cancels the bypass offset (`target_lateral_offset = 0.0`) and stops in place.
   * **Rationale:** Prevents the robot from colliding with or scraping against side walls while shifting laterally to avoid a pedestrian in a narrow corridor.

4. **Symmetric Centerline Return Path & Absolute Rotation Alignment**
   * **Change:** Anchored the return path coordinate calculation to the absolute starting pose (`self.init_x`, `self.init_y`, `self.init_yaw`) recorded at the beginning of the run, rather than recording a new coordinate frame from a drifted turnaround pose. Replaced Euclidean distance checks with longitudinal projection along the path, and locked rotation targets to absolute orientations (`init_yaw` and `init_yaw + pi`).
   * **Rationale:** If the robot drifted laterally during the forward run (e.g. to bypass a chair), recording its drifted position at Waypoint B as the new return path origin resulted in a shifted return path, causing it to take a different trajectory and hit obstacles. Anchoring to the original centerline and aligning rotation target headings absolutely forces the robot to correct any accumulated drift, guaranteeing it returns along the exact same track to its start mark.

5. **ML Coordinate Mapping Correction (Predictive Mode Random Strafing Fix)**
   * **Change:** Refactored the window feature construction in `velocity_estimator.py` to map `cz` (depth) to `rx` (Robot X / forward) and `-cx` (inverted horizontal coordinate) to `ry` (Robot Y / left). 
   * **Rationale:** The trained MLP expected positions in standard relative robot-frame coordinates `[Robot X, Robot Y] = [depth, -camera_x]`. However, the live code was passing raw camera plane horizontal `cx` and vertical `cy` (height) while completely leaving out depth. This feature-space mismatch led the MLP to predict chaotic, non-zero speeds for static obstacles when the robot was in motion, triggering random strafing in predictive mode. Aligning the coordinate space makes predictions stable and highly accurate.

6. **Adaptive Avoidance Corridors**
   * **Change:** Implemented dynamic corridors in `ab_comparison_test.py` based on whether the camera detects a moving pedestrian (`speed > 0.15 m/s`). If a dynamic pedestrian is detected, it uses a wide avoidance corridor and bypass offset (`AVOIDANCE_LATERAL = 0.45 m`, `BYPASS_OFFSET = 0.4 m`, `LATERAL_CORRIDOR = 0.45 m`). Otherwise (for static objects like chairs or table legs, or when in reactive mode), it uses a tighter envelope (`AVOIDANCE_LATERAL = 0.3 m`, `BYPASS_OFFSET = 0.25 m`, `LATERAL_CORRIDOR = 0.3 m`).
   * **Rationale:** Permits the robot to ignore static objects that are already far enough to the side to be cleared safely (preventing overcorrection), while executing extremely tight shifts (`0.25 m`) to pass objects directly in the path without colliding with side walls.

**Status:** Completed 🟢. Fully deployed to the robot in the robotics lab and verified starting up successfully.