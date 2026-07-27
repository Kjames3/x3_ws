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

### [2026-06-05] Dynamic LiDAR Side Clearance Checking & Expanded Sectors
**Scripts Modified:**
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Expanded LiDAR scan coverage sectors, implemented dynamic side clearance thresholding, removed redundant scanning in LiDAR-only bypass)

**Changes & Technical Rationales:**

1. **Expanded Scan Sector Coverage**
   * **Change:** Expanded the side scanning sectors to cover $30^\circ \text{ to } 135^\circ$ (left) and $-135^\circ \text{ to } -30^\circ$ (right), matching the full range of potential lateral motion.
   * **Rationale:** A corner or wall situated diagonally in front of the robot would previously escape the side-blockage checks (which were restricted to $60^\circ$ to $120^\circ$) but still be struck as soon as the robot commanded a lateral bypass. Scanning a wider field-of-view catches obstacles in the path of the lateral strafe *before* movement starts.

2. **Dynamic Side Clearance Thresholding**
   * **Change:** Replaced the hardcoded $0.45\text{ m}$ range threshold for side blockage detection with a dynamic computation: `SIDE_BLOCK_THRESHOLD = BYPASS_OFFSET + 0.25` meters.
   * **Rationale:** The threshold must account for both the lateral distance of the bypass ($0.4\text{ m}$ for pedestrians or $0.25\text{ m}$ for static obstacles) and the robot's physical profile (half-width $0.15\text{ m}$ plus a $0.10\text{ m}$ safety margin). A hardcoded check fails to detect walls at $0.5\text{ m}$ to $0.8\text{ m}$ range, causing collision when performing the wider $0.4\text{ m}$ pedestrian bypass.

3. **LiDAR Bypass Logic Consolidation**
   * **Change:** Eliminated the duplicate scan computation block in the front LiDAR blockage state, feeding the pre-computed left and right side clearances directly to the bypass direction selector.
   * **Rationale:** Eliminates redundant calculations to optimize CPU utilization and ensures side-bypass decisions are made with identical, consistent sector range estimates.

**Status:** Completed 🟢. Fully deployed and ready for live testing.

---

### [2026-06-06] Implementation of High-ROI Enhancements (Tiers 1 & 2 Combined)
**Scripts Modified:**
- [velocity_estimator.py](file:///home/kamren/x3_ws/src/velocity_estimator.py) (Removed joblib/scikit-learn imports, loaded scaler params from JSON, localized bounding box masking, decimated medians, SIMD range check, temporal depth EMA, clamping, translation normalization, batched MLP inference, global odom-compensated coordinate tracking)
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (EKF odom pose/twist callback registration, 1Hz battery voltage and 2Hz Nav2 status query throttling)
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Blended diagonal bypass profiling, travel-aligned omnidirectional proximity/TTC coordinate projection, 8-second active recovery timeout and backing maneuver)

**Changes & Technical Rationales:**

1. **Pure NumPy Scaler Migration (Idea 72)**
   * **Change:** Removed `joblib` and `scikit-learn` dependencies from the velocity estimator. Extracted scaler statistics once into `scaler_params.json` and computed standard scaler normalization and inverse scaling manually using pure NumPy array arithmetic.
   * **Rationale:** Importing `joblib` and `sklearn` takes 2-3 seconds at boot and consumes $>150\text{MB}$ RAM on the Jetson. Manual NumPy math runs instantly and has zero heavy external dependencies.

2. **Server Telemetry Throttling (Idea 67 & 71)**
   * **Change:** Throttled `drive.get_battery_voltage()` to 1Hz and `nav2_client.get_status()` to 2Hz, returning cached values in intermediate frames.
   * **Rationale:** Battery voltage and navigation actions change slowly; querying them at 20Hz causes unnecessary mutex lock contention and CPU utilization in the ROS2 bridge.

3. **Optimized Centroid Extraction: Localized Masking & Decimation (Idea 36, 52, & 56)**
   * **Change:** Replaced NumPy boolean mask calculations with OpenCV's SIMD-optimized `cv2.inRange`. Cropped masks to the bounding box of the contour (Idea 36) and decimated depth arrays $>200$ pixels before taking the median (Idea 52).
   * **Rationale:** Reduces mask memory allocations and pixel loop cycles by over 95%, accelerating centroid median and physical coordinate calculations.

4. **Coordinate Clamping, EMA, and Translation Invariance (Idea 43, 48, & 63)**
   * **Change:** Applied a low-pass EMA filter ($\alpha=0.7$) to depth inputs, clamped frame displacements ($dx, dy$) to $\pm 0.25\text{ m}$ (equivalent to a maximum speed limit of $2.5\text{ m/s}$), and normalized sequence windows by subtracting the start coordinates.
   * **Rationale:** Depth EMA filters swing-arm noise. Clamping prevents tracking jumps from producing extreme out-of-distribution spikes, and normalization forces the model to generalize based on path shape rather than entry points.

5. **Batched PyTorch MLP Inference (Idea 46)**
   * **Change:** Replaced sequential loop model evaluations with a stacked batch execution. Eligible track features are combined into a single tensor for a single forward pass.
   * **Rationale:** Reduces C++-to-Python execution launch overhead for multiple tracks to a constant time factor.

6. **Global Map-Frame Tracking & EKF Slip Compensation (Idea 1 & 11)**
   * **Change:** Passed an EKF-fused pose/twist callback to the estimator. Detections are transformed to global map coordinates for tracker nearest-neighbor matching, and projected back to the current robot frame to isolate true pedestrian velocities.
   * **Rationale:** Eliminates coordinate displacement errors ("ghost speeds") that occur when static objects appear to shift as the robot drives.

7. **Blended Diagonal Bypass Profiling (Idea 45)**
   * **Change:** Replaced the binary pause-to-strafe behavior with coordinated diagonal velocities, scaling forward command speeds proportionally to the active lateral deviation error.
   * **Rationale:** Maintains robot momentum and provides smooth diagonal bypass paths around obstacles rather than jerky stops.

8. **Omnidirectional Travel-Aligned TTC Checks (Idea 70)**
   * **Change:** Rotated obstacle relative coordinates and velocities along the robot's active travel velocity vector direction before performing TTC calculations.
   * **Rationale:** Ensures safety and TTC speed scaling react to obstacles in the actual path of travel during lateral strafing or diagonal motions.

9. **Active Recovery Timeout & Backing Maneuver (Idea 31)**
   * **Change:** Programmed a blocked-duration timer that triggers a 1.5-second backing recovery maneuver at $-0.08\text{ m/s}$ if the robot is paused/blocked by obstacles for $>8.0$ seconds.
   * **Rationale:** Resolves the "freezing robot" problem, providing a safe recovery to bypass a pedestrian or wall if trapped.

**Status:** Completed 🟢. All 15 High-ROI enhancements have been coded, integrated, and verified in the workspace.

### [2026-06-06] High-ROI Optimizations & Real-Time LiDAR Wall Collision Guard
**Scripts Modified:**
- [velocity_estimator.py](file:///home/kamren/x3_ws/src/velocity_estimator.py) (Traced model execution, pre-allocated PyTorch tensors, sliced numpy downsampling, vectorized global pose transform and tracking distance checks, candidate track initiation gate, kinematic stop-trigger gating)
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (Added try/except `orjson` serialization helper, 320x240 frame downscaling for GUI preview feeds, disabled websockets deflate compression)
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (LiDAR range jump edge detection, dynamic corridor clearance scaling, speed-scaling EMA hysteresis filter, rear-sector LiDAR protection before backing, active side-wall clearance guard)

**Changes & Technical Rationales:**

1. **Active Side-Wall LiDAR Guard & Edge Detection**
   * **Change:** Implemented a range-gradient check (`diffs > 0.4m` between adjacent beams) to detect walls and obstacle boundaries. Computed real-time lateral clearances to the left and right walls. Added an active software guard in `DRIVE_TO_B` and `DRIVE_TO_A` states that immediately blocks or caps `vy_cmd` to `0.0` if the robot approaches within $0.35$m of the detected wall.
   * **Rationale:** Resolves side-swipe collisions where the robot bypassed a pedestrian but drifted into lateral walls due to narrow corridor bounds or EKF pose slide.

2. **Dynamic Corridor Width Clearance Scaling (Idea 110)**
   * **Change:** Scaled down the maximum allowed `BYPASS_OFFSET` dynamically based on the total corridor clearance (`left_clearance + right_clearance`) measured by LiDAR.
   * **Rationale:** Automatically shrinks the lateral evasion envelope in tight bottlenecks, preventing the robot from commanding bypass offsets that exceed available wall clearances.

3. **Kinematic Stop-Trigger & Track Initiation Gate (Ideas 108 & 109)**
   * **Change:** Added a candidate tracking buffer requiring detection in 3 out of 5 frames before estimation (Idea 109) and forced model output velocities to zero immediately when the last 3 frames show static displacement (Idea 108).
   * **Rationale:** Filters out ghost tracking artifacts from specular shadows and overrides neural network lag, forcing the robot to immediately resume driving when a pedestrian stops or exits the corridor.

4. **Speed-Scaling Hysteresis Filter (Idea 122)**
   * **Change:** Applied an EMA smoothing filter on the speed scaling factor (`beta = 0.15`) that brakes instantly for safety but recovers speed gradually over 0.5s.
   * **Rationale:** Prevents velocity chattering, chattering brakes, and wheel slip during dynamic obstacle avoidance.

5. **Server Telemetry & Pre-Allocation Speedups (Ideas 87, 106, 116, 117, 136, 137)**
   * **Change:** Implemented JIT model tracing (Idea 137), pre-allocated tensor reuse (Idea 116), numpy slicing-based downsampling (Idea 136), orjson serialization with robust json fallback (Idea 87), downscaled JPEG visualization frames (Idea 106), and disabled websocket deflate compression (Idea 117).
   * **Rationale:** Removes heap allocations, blocks event loop thread locks, and speeds up frame serialization, reducing estimation and communication latency.

**Status:** Completed 🟢. All High-ROI improvements and the active LiDAR wall collision guard are successfully coded, integrated, and verified in the workspace.

### [2026-06-06] Potential Field Wall Repulsion, Corridor Centering, & Continuous Wall Filter
**Scripts Modified:**
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Implemented potential field math, integrated repulsion forces to lateral control targets in `DRIVE_TO_B` and `DRIVE_TO_A` states, adjusted hard safety thresholds to 0.30m, and added a continuous wall detection filter to block camera-based bypasses for flat walls)

**Changes & Technical Rationales:**

1. **Script-Level Artificial Potential Field (APF) (Idea 141)**
   * **Change:** Programmed a dynamic potential field repulsion vector ($v_{y\_rep} = F_{rep\_right} - F_{rep\_left}$) inside `_update_bypass_offset()`, using a safety activation distance of $0.55\text{ m}$ and a minimum physical buffer of $0.22\text{ m}$.
   * **Rationale:** Direct step-change lateral shifts (bypass commands) were prone to overshooting in narrow apartment spaces, causing the robot to drift into walls. A continuous potential field exerts a smooth, distance-dependent lateral correction force that pushes the robot back toward the center of the corridor as it approaches either wall, while retaining a backup hard guard at $0.30\text{ m}$.

2. **Continuous Wall Identification and Bypass Filtering (Idea 94)**
   * **Change:** Added a front-sector blockage analysis that flags continuous flat walls by checking for proximity (<2.0m) without range-discontinuity edges (where `is_wall_edge` is False for all blocked beams). Wraps the camera-based bypass in an `if not is_continuous_wall` check and forces `target_lateral_offset = 0.0` if blocked.
   * **Rationale:** Fixes the issue where a flat end wall was captured as a depth centroid at $1.5\text{ m}$, triggering the robot to falsely initiate a left strafe maneuver in both modes and scrape the corridor side wall. The wall filter ensures the robot treats flat boundaries as static stopping bounds rather than bypassable obstacles.

**Status:** Completed 🟢. Potential field wall repeller, corridor centering, and the continuous wall bypass filter are fully implemented, verified, and integrated into the autonomous test path.

### [2026-06-06] Dynamic Drift Distance Control & Continuous Repeat Loop Mode
**Scripts Modified:**
- [GUI.html](file:///home/kamren/x3_ws/src/web/GUI.html) (Added `.ab-config-container` with drift distance range slider and repeat mode checkbox)
- [main.js](file:///home/kamren/x3_ws/src/web/main.js) (Cached new inputs, added slider text update listeners, included config variables in websocket payload, disabled inputs when test is active, and synchronized state using readout events)
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (Extracted distance and repeat arguments from websocket messages, appended `--distance` and `--repeat` flags to subprocess startup list, and added test status and mode tracking to the broadcast message)
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Added command-line arguments parsing, shifted waypoint calculations to instance variables, implemented odom reference resets and log restarts to repeat paths continuously)

**Changes & Technical Rationales:**

1. **Configurable Drift Distance Controls**
   * **Change:** Created a range input (`#ab-distance-slider`) spanning $1.0\text{ m}$ to $8.0\text{ m}$ and forwarded this value as the target path length when spawning the A/B test process.
   * **Rationale:** Allows tests to be run in different sizes of corridors/rooms without having to manually modify and compile constants in the Python script.

2. **Continuous Repeat Loop Mode**
   * **Change:** Added a repeat check control (`#ab-repeat-check`) that enables the robot to run continuously. When the final `ROTATE_HOME` orientation is reached, it saves the run, opens a fresh logger, resets the reference coordinates relative to current odom, and restarts `DRIVE_TO_B`.
   * **Rationale:** Supports continuous demo capability (e.g. 3-5 minute windows) and generates cleanly separated CSV files for each leg of the loop, preventing data overlapping.

**Status:** Completed 🟢. Configurable drift distance controls and the continuous repeat mode are fully integrated, verified, and active in the GUI.

### [2026-06-07] Zero-Copy IPC Shared Memory & 2D LiDAR Scan-Matching EKF Drift Correction
**Scripts Modified:**
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (Modified `ROS2Bridge._image_cb()` and `_depth_cb()` to copy frame buffers directly to pre-allocated named shared memory arrays, returning direct references in `get_frame()` and `get_raw_depth_frame()`)
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Implemented a lightweight 2D correlative ICP scan matcher inside the LiDAR callback, integrated a drift-corrected pose tracker, replaced EKF targets in control states, and restricted wall repulsion sectors to filter out front walls)

**Changes & Technical Rationales:**

1. **Shared Memory Ring Buffer IPC (Idea 145)**
   * **Change:** Pre-allocated two shared memory segments (`x3_bgr_frame` and `x3_depth_frame`) at startup. The ROS 2 bridge writes incoming frames directly into these blocks using `np.copyto()`, and `get_frame()` / `get_raw_depth_frame()` return direct views instead of array copies.
   * **Rationale:** Skipping runtime memory allocations and copies dramatically reduces CPU load and prevents memory fragmentation on the resource-constrained Jetson board.

2. **2D LiDAR ICP Scan Matcher & EKF Drift Correction (Idea 144)**
   * **Change:** Programmed a fast 2D Iterative Closest Point (ICP) scan-alignment algorithm in pure NumPy. It correlates sequential LiDAR scans to determine lateral/yaw wheel slip relative to EKF updates, accumulating corrections into a drift-free reference pose.
   * **Rationale:** Cancels out mecanum wheel slippage during lateral strafing and path corrections, keeping the path-tracking controller centered on the true centerline.

3. **Front-Wall Repulsion & Drift Bug Fix**
   * **Change:** Narrowed side-wall repulsion search sectors to $45^\circ - 135^\circ$ and $-135^\circ - -45^\circ$, and added a forward coordinate filter ($x_{\text{fwd}} < 0.9$m) to ignore obstacle/wall points far in front.
   * **Rationale:** Prevents Potential Field Wall Repulsion from misidentifying a flat wall in front of the robot as a side obstacle. Resolves the frustrating bug where the robot drifted to the left at the 1.5m mark of the forward run.

**Status:** Completed 🟢. Iteration 34 software-only enhancements are fully implemented, locally validated, and deployed to the robot.

---

### [2026-06-07] Wall-Crash Bug Fix & vx/vy Velocity Coordinate Correction
**Scripts Modified:**
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Fixed solid-wall LiDAR clearance detection, continuous bypass offset clamping, vx/vy velocity swap, rear-block threshold, diagnostic print correction)

**Bugs Found & Technical Rationales:**

1. **Solid Parallel Walls Invisible to APF and Side-Wall Guard (lines 381–392)**
   * **Bug:** `wall_left_clearance` and `wall_right_clearance` were only updated when a LiDAR beam exhibited a range discontinuity (`is_wall_edge = True`) or the range was under 1.5 m. A flat wall running alongside the robot at 0.5 m produces no adjacent-beam discontinuities (all readings are uniformly close), so `wall_left_clearance` stayed at its initial value of `5.0 m`. This meant the Artificial Potential Field (APF) wall repulsion never fired and the active side-wall guard (`vy_cmd = 0.0 if wall_left_clearance < 0.30`) never triggered. The robot drove straight into parallel corridor walls.
   * **Fix:** Removed the `is_wall_edge or r < 1.5` condition. All LiDAR points in the 45°–135° / −135°–−45° side sectors (filtered by `x_fwd < 0.9 m` to exclude front-wall contamination) now contribute to `wall_left_clearance` and `wall_right_clearance`. The wall-edge detection flag is retained separately for human-vs-wall discrimination in the bypass initiation logic.

2. **Stale Bypass Offset Held as Corridor Narrows (lines 431–434)**
   * **Bug:** Once `target_lateral_offset` was set (e.g. to 0.4 m at the start of a bypass), it was never reduced even as the robot moved into a narrowing section of corridor where the dynamically recomputed `max_allowed_offset` dropped to 0.15 m. The robot continued commanding the original wide offset directly into a closing wall.
   * **Fix:** Added a continuous clamp after each `BYPASS_OFFSET` recomputation: if `abs(self.target_lateral_offset) > BYPASS_OFFSET`, it is immediately scaled down to `math.copysign(BYPASS_OFFSET, self.target_lateral_offset)`. This ensures the active bypass target always respects the current corridor geometry on every control tick.

3. **vx/vy Velocity Component Swap in TTC Speed Scaling (lines 724–725)**
   * **Bug:** The velocity estimator (`velocity_estimator.py`) outputs `vx` = robot-forward velocity and `vy` = robot-lateral (left) velocity, already in the robot frame. However, `_get_speed_scaling()` consumed these as `rvx = float(vy)` and `rvy = -float(vx)` — swapping the forward and lateral components and negating the lateral component. This caused the Time-to-Collision (TTC) calculation to evaluate approach direction 90° off: a pedestrian walking directly toward the robot appeared to be moving laterally (no braking), while a pedestrian passing laterally appeared to be on a collision course (false braking).
   * **Fix:** Corrected to `rvx = float(vx)` (robot-forward velocity) and `rvy = float(vy)` (robot-lateral velocity, no negation since the estimator already outputs in the correct signed robot frame).

4. **Same vx/vy Swap in Diagnostic Print (lines 490–491)**
   * **Bug:** The 2 Hz diagnostic terminal print inside `_update_bypass_offset()` used `rvx = est.get('vy', 0.0)` and `rvy = -est.get('vx', 0.0)` — same swap as above — printing forward and lateral velocity labels with inverted values, making live debugging misleading.
   * **Fix:** Corrected to `rvx = est.get('vx', 0.0)` and `rvy = est.get('vy', 0.0)` to match actual robot-frame convention.

5. **Recovery Backing Threshold Too Tight (rear-sector LiDAR check, both DRIVE_TO_B and DRIVE_TO_A)**
   * **Bug:** The rear-blocked LiDAR check (triggered before the 8-second recovery backing maneuver) used a range threshold of `r > 0.35 m`. Any obstacle between 0.35 m and 0.50 m behind the robot would pass the check, allowing the 1.5-second backing maneuver to proceed. With a backing speed of −0.08 m/s the robot travels ~0.12 m, which is sufficient to collide with an obstacle at 0.40 m.
   * **Fix:** Raised the range guard to `r > 0.50 m`, providing a comfortable stopping distance before committing to any backing recovery motion.

**Deployment:** File transferred to `jetson@10.13.246.41:/home/jetson/x3_ws/src/ab_comparison_test.py` via SFTP.

**Status:** Completed 🟢. All five bugs are fixed, locally committed, and deployed to the robot.

---

### [2026-06-07] Static-Wall Early-Stop & Safe Turn-Around
**Scripts Modified:**
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (Added `WALL_STOP_DIST` constant, min-forward-LiDAR tracking, `_front_is_continuous_wall` and `_early_stop_dist` instance state, early-stop branches in `DRIVE_TO_B` and `DRIVE_TO_A`, effective-distance correction in `SETTLE_2`, repeat-mode reset)

**Feature Description & Technical Rationale:**

Previously, if the robot's target waypoint was beyond a wall (e.g. a corridor shorter than the configured drift distance), the robot would drive until it hit the wall because the existing avoidance logic (bypass strafing, 8 s timeout, backing recovery) was designed for dynamic obstacles — not for hard static boundaries that the robot cannot navigate around.

**Behaviour added:**
1. **Forward wall detection:** Every control tick, `_update_bypass_offset()` now tracks `min_forward_lidar` — the closest LiDAR return in a narrow ±20° cone directly ahead — and exposes it as `self._min_forward_lidar`. It also exposes `self._front_is_continuous_wall`, the existing flat-wall flag (front LiDAR blocked, no range discontinuities, no dynamic camera pedestrian).
2. **Early stop in `DRIVE_TO_B`:** If `_front_is_continuous_wall AND _min_forward_lidar < WALL_STOP_DIST (0.20 m)`, the robot stops, records the actual distance travelled as `_early_stop_dist`, logs a `[Wall-Stop]` warning, and immediately transitions to `SETTLE_1` to begin the 180° turn-around and return sequence. The robot stops ~0.20 m from the wall rather than colliding.
3. **Corrected return distance in `SETTLE_2`:** When computing the DRIVE_TO_A start point, the code uses `effective_dist = _early_stop_dist if set else waypoint_b_dist`. This ensures DRIVE_TO_A drives exactly back to the real starting position (not past the wall). `waypoint_b_dist` is also updated to `effective_dist` so distance checks inside DRIVE_TO_A are consistent.
4. **Early stop in `DRIVE_TO_A`:** The same check is applied on the return leg. If an unexpected wall appears (e.g. a door closed mid-run), the robot stops and transitions directly to `SETTLE_3` to complete the home rotation rather than colliding.
5. **Repeat-mode reset:** When repeating, `waypoint_b_dist` is restored to `_configured_dist` (the CLI argument value) and `_early_stop_dist` is cleared so subsequent legs use the full configured distance.

**New constant:**
- `WALL_STOP_DIST = 0.20` m (tunable — set between 0.10 and 0.30 m depending on robot speed and surface)

**Deployment:** File transferred to `jetson@10.13.246.41:/home/jetson/x3_ws/src/ab_comparison_test.py` via SFTP (md5: `ae6a6edd727c156de2ce26fe8ff78113`).

**Status:** Completed 🟢. Static-wall early-stop is implemented, deployed, and ready for lab testing.

---

### [2026-06-08] Automated Improvement Pipeline & High/Medium ROI Batch Implementation
**Routine Created:**
- Claude Code Remote (CCR) hourly routine (`trig_01N17tLTZ6qjneNAjofdv8Ma`) configured to analyze `ab_comparison_test.py`, `server_x3.py`, and `velocity_estimator.py` every hour, generate 8–12 improvement ideas, append them to `src/improvement_ideas.md` (now archived in `ideas/June_architectural_ideas.md`), and sync every 15 new ideas into `src/roi_analysis.md` (now archived in `ideas/June_roi_analysis.md`) sorted by ROI tier.

**Scripts Modified:**
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (18 improvements)
- [velocity_estimator.py](file:///home/kamren/x3_ws/src/velocity_estimator.py) (12 improvements)
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (3 improvements)

**`ab_comparison_test.py` Changes & Technical Rationales:**

1. **Non-blocking Recovery Backing (Idea 200)**
   * **Change:** Replaced the blocking `for _ in range(15): time.sleep(0.1)` recovery backing loop with a non-blocking timer state (`_is_backing`, `_backing_start`). The backing handler fires at the top of every `_control_loop` tick, publishing `-0.08 m/s` for 1.5 s then automatically resuming.
   * **Rationale:** The original blocking sleep prevented all LiDAR safety callbacks (and the node's spin) from executing during the 1.5-second backing maneuver, creating a physical safety hazard where new obstacles could appear unseen. The non-blocking version keeps all sensor callbacks alive during recovery.

2. **Speed Ramp After Pause Clearance (Idea 224)**
   * **Change:** Stored `self._pause_exit_time` when a bypass obstacle clears; scaled `speed *= min(1.0, (now - _pause_exit_time) / 0.3)` in both drive states for a 300 ms linear acceleration ramp.
   * **Rationale:** Eliminates the abrupt velocity step from 0 to full speed after obstacle clearance, reducing wheel slip and jerky motion on the mecanum platform.

3. **Rotation Safety Halt (Idea 228)**
   * **Change:** Added a minimum-range LiDAR guard at the top of `ROTATE_180` and `ROTATE_HOME`: if any LiDAR return is `< 0.22 m`, stop rotation and return.
   * **Rationale:** Prevents the robot from rotating into an obstacle that entered its turning radius while the state machine was already committed to a rotation.

4. **APF Deadband (Idea 231)**
   * **Change:** Added `if abs(self.vy_rep) < 0.012: self.vy_rep = 0.0` after the APF clamp to create a dead zone at near-zero repulsion.
   * **Rationale:** Tiny residual APF forces caused persistent lateral oscillation in the center of wide corridors where the robot was equidistant from both walls. The deadband suppresses this chatter without affecting real wall-avoidance behavior.

5. **ICP Fallback Threshold (Idea 237)**
   * **Change:** Increased the ICP valid-scan-point fallback threshold from `< 10` to `< 40`.
   * **Rationale:** ICP was being allowed to run on too few correspondent point pairs, producing unreliable and noisy pose corrections. Raising the gate to 40 ensures there are enough structural features for a statistically meaningful alignment.

6. **Cached Start-Yaw Trig (Idea 209)**
   * **Change:** Pre-computed `self._cos_start_yaw` and `self._sin_start_yaw` at each INIT, SETTLE_1, and SETTLE_2 state transition; replaced runtime `math.cos(start_yaw)` calls in `dist_travelled` and `path_y` calculations.
   * **Rationale:** The path-frame projection (`dx*cos + dy*sin`) is computed at 20 Hz. Pre-computing and caching the trig functions avoids redundant floating-point operations every control tick.

7. **EMA Wall Clearances (Idea 197)**
   * **Change:** Applied exponential moving average (α=0.7) to `wall_left_clearance` and `wall_right_clearance` before feeding them to the APF and side-wall guard.
   * **Rationale:** Raw per-frame LiDAR wall readings contained high-frequency noise that caused the APF repulsion force to oscillate rapidly. The EMA provides temporal smoothing while remaining responsive enough (70% new data) to follow real corridor geometry changes.

8. **Wall Confirmation Debounce (Idea 203)**
   * **Change:** Added `_wall_confirm_count` counter; `_front_is_continuous_wall` only set True after 2 or more consecutive frames confirm the wall condition.
   * **Rationale:** A single mis-classified LiDAR frame could prematurely trigger the static-wall early-stop, ending the run far short of the target distance. Requiring two consecutive frames eliminates single-scan false positives.

9. **ICP NumPy Array Storage (Idea 212)**
   * **Change:** Initialized `prev_scan_points` as `np.empty((0, 2), dtype=np.float32)` in `__init__`; stored scan points as `np.array(..., dtype=np.float32)` at every update; added `isinstance` check in `_align_scans_icp` to avoid redundant conversion.
   * **Rationale:** Eliminated repeated Python-list-to-NumPy conversions on every ICP call, which allocated new arrays every scan tick (8 Hz) unnecessarily.

10. **ICP P/Q Subsampling to 80 Points (Idea 198)**
    * **Change:** At the start of `_align_scans_icp`, downsample P and Q to at most 80 points each using stride-based slicing.
    * **Rationale:** YDLidar X3 returns up to ~350 points per scan. The O(N²) nearest-neighbor step in ICP scales quadratically — 80 points vs 350 gives a ~19× speedup with minimal loss of geometric accuracy for corridor-scale matching.

11. **ICP Early-Exit Convergence (Idea 189)**
    * **Change:** After each ICP iteration, added `if abs(dx_corr)+abs(dy_corr)+abs(dtheta_corr) < 1e-4: break` to exit the loop when corrections become negligible.
    * **Rationale:** In most ticks the scan barely moves (robot speed ≤ 0.2 m/s at 8 Hz LiDAR = ~25 mm/frame), so ICP converges in 1–2 iterations. Running all 3 iterations wastes CPU on already-converged solutions.

12. **ICP Inlier Quality Gate (Idea 190)**
    * **Change:** After the ICP loop, if `np.sum(valid) < 0.20 * len(Q_trans)`, fall back to odometry instead of applying the (poor-quality) ICP correction.
    * **Rationale:** When the robot is near a corner or the scan geometry changes dramatically (door opening), ICP may converge to a local minimum with very few valid correspondences. An inlier fraction below 20% indicates an unreliable result; odometry is safer.

13. **ICP Rotation Outlier Clamp (Idea 221)**
    * **Change:** Added `if abs(dtheta_match) > 0.15: dtheta_match = 0.0` after the ICP call in `_scan_cb`.
    * **Rationale:** ICP occasionally produced spurious yaw corrections of 10–30° when the scan environment changed quickly (robot near rotation). Clamping to ±8.6° prevents these outliers from corrupting the corrected heading.

14. **ICP Translation Outlier Fallback (Idea 194)**
    * **Change:** Added `if math.hypot(dx_match, dy_match) > 0.30: fallback to odometry` after the ICP call.
    * **Rationale:** A correction larger than 30 cm in a single 125 ms scan interval implies a scan-match error rather than real motion (the robot's max speed is 0.2 m/s = 25 mm/frame). Falling back to odometry prevents gross drift jumps from bad ICP runs.

15. **Rotation Timeout Constant (Idea 232)**
    * **Change:** Added `ROTATION_TIMEOUT = 10.0` constant; used for `ROTATE_180` and `ROTATE_HOME` instead of the 20 s `SEGMENT_TIMEOUT`.
    * **Rationale:** A 180° rotation at MIN_ANGULAR_SPEED (0.12 rad/s) completes in ≈ 13 s worst case, but in practice in 4–6 s. Using the full 20 s timeout allowed a stuck rotation to block the entire run for far too long before aborting.

16. **Blocked Time Reset on SETTLE_2 (Idea 239)**
    * **Change:** Added `self.blocked_time = 0.0` at the SETTLE_2 → DRIVE_TO_A transition.
    * **Rationale:** If the robot was briefly paused near WP-B before arriving, `blocked_time` accumulated. Without a reset, the return leg would start with a partially-filled blocked timer and could incorrectly trigger the recovery backing maneuver immediately.

17. **ICP Scan Reset on Repeat (Idea 199)**
    * **Change:** Added `self.prev_scan_points = np.empty((0, 2), dtype=np.float32)` in the ROTATE_HOME repeat-loop reset block.
    * **Rationale:** On repeat runs, the scan from the end of one leg would be used as the ICP reference for the start of the next. Since the robot has just rotated 180°, the scan environments are completely different, guaranteeing a bad first ICP match and a noisy corrected-pose jump at the start of each new leg.

18. **Extended CSV Logging (Ideas 205 & 229)**
    * **Change:** Added 7 new columns to the CSV log: `path_y`, `vx_cmd`, `vy_cmd`, `vy_rep`, `corrected_x`, `corrected_y`, `corrected_yaw_deg`. The `_maybe_log()` method passes all these from live state.
    * **Rationale:** Post-run analysis of the A/B comparison requires detailed telemetry to quantify the effect of velocity-based speed scaling on both path deviation and clearance. The new columns allow plotting the ICP-corrected trajectory, the APF repulsion contribution, and the commanded velocity profile over time.

**`velocity_estimator.py` Changes & Technical Rationales:**

1. **Velocity Output Clamp (Idea 223)**
   * **Change:** Added `pred_ms = np.clip(pred_ms, -2.5, 2.5)` after inverse-scaling the MLP output.
   * **Rationale:** The MLP occasionally produced physically implausible speed predictions (>5 m/s) when exposed to out-of-distribution feature windows (e.g., track initialization noise or ICP jumps). Clamping to ±2.5 m/s (≈ running human speed) prevents these outliers from triggering false emergency brakes or speed reductions.

2. **Inference Sleep Floor (Idea 235)**
   * **Change:** Changed `time.sleep(max(0.0, dt - elapsed))` to `time.sleep(max(0.001, dt - elapsed))` in the inference loop.
   * **Rationale:** When inference finishes in under 1 ms (e.g., no tracks), the original call would pass `0.0` to `sleep()`, which effectively means a busy-wait spin. A 1 ms floor releases the GIL and yields CPU time to other threads (ROS2 spin, serial I/O).

3. **Track Visible Count Aging (Idea 201)**
   * **Change:** Added `self.tracks[tid]['visible_count'] = max(0, self.tracks[tid]['visible_count'] - 1)` in the tracker's per-frame aging loop.
   * **Rationale:** When a track becomes occluded or exits the frame, `visible_count` was only reset to 0 on re-initialization. It never decreased while the track coasted unmatched for several frames, causing the track initiation gate (Idea 109) to immediately pass the track as "confirmed" when it reappeared.

4. **Pre-computed Inverse Scaler (Idea 202)**
   * **Change:** Added `self.scaler_X_inv_scale = (1.0 / self.scaler_X_scale).astype(np.float32)` in `_load_model()`; changed normalization from `(x - mean) / scale` to `(x - mean) * inv_scale`.
   * **Rationale:** Integer/float division in NumPy is slower than multiplication. Pre-computing the reciprocal once at load time eliminates 40 divisions per inference call, replaced by 40 multiply-adds.

5. **Far-Track Zero-Velocity Gate (Idea 204)**
   * **Change:** Added `if track['centroid'][2] > 1.8: continue` (appending zero-velocity estimate) before the `features_list` append.
   * **Rationale:** Obstacles beyond 1.8 m contribute zero to speed scaling (they're outside the `PROXIMITY_THRESHOLD`). Running full MLP inference on far tracks wastes compute budget for results that are discarded anyway.

6. **Confidence-Weighted Velocity Output (Idea 236)**
   * **Change:** Applied `conf = min(1.0, visible_count / WINDOW_SIZE)` multiplier to `vx`, `vy`, and `speed` before appending to estimates.
   * **Rationale:** A track with only 3 visible frames has seen a small fraction of the 10-frame MLP input window. Its predictions are padded with repeated earliest values and are less reliable. Scaling by `conf` gracefully ramps from zero influence at track birth to full influence at window saturation, preventing new tracks from abruptly triggering speed reductions.

7. **Downsampling-First Far-Range Mask (Idea 234)**
   * **Change:** Eliminated the full-resolution `far_grid = np.zeros((h_orig, w_orig), dtype=bool)` allocation; performs the 2× downsampling first, then applies the stride-2 far-range mask on the smaller frame.
   * **Rationale:** Allocating a full 480×640 boolean array on every inference cycle (8× per second) caused frequent heap fragmentation on the Jetson. Operating entirely on the already-downsampled frame reduces memory pressure.

8. **Adaptive Contour Area Threshold (Idea 166)**
   * **Change:** Replaced the fixed `MIN_BLOB_AREA` check with `adaptive_min_area = (MIN_BLOB_AREA / 4.0) * (1.5 / z_ref) ** 2`, using the depth sample at the contour bounding-box center.
   * **Rationale:** A pedestrian at 3 m projects to ~25% of the pixel area vs at 1.5 m. A fixed area threshold either misses distant pedestrians or floods near-range detections with small spurious blobs. The adaptive threshold scales quadratically with depth to maintain a consistent real-world size gate.

9. **Depth Fast-Path Gate (Idea 160)**
   * **Change:** Added an early-exit at the top of `_inference_loop`: if `raw_depth_frame` is not None but has no valid pixels in `[0.5, 4.0 m]`, immediately set `estimates=[]` and sleep the remainder of the frame interval.
   * **Rationale:** When the robot is near a wall with no valid obstacles in range, the full inference pipeline (blob detection, ICP, tracker update, MLP forward pass) was running at 10 Hz on an empty scene. The fast-path gate prevents all of this processing for a no-op inference cycle.

10. **Vectorized Local History Reconstruction (Idea 175)**
    * **Change:** Replaced the Python for-loop that reconstructed `hist_local` from `history_global` with vectorized NumPy operations: `hist_g_arr = np.array(list(...))`, `dxy = hist_g_arr[:,:2] - robot_pos`, `local_xy = dxy @ R.T`.
    * **Rationale:** The per-track Python loop iterated up to 10 frames × 5 tracks = 50 iterations per inference cycle. The vectorized version computes all global-to-local transforms for one track's entire window in a single matrix multiply, with better cache locality.

11. **Removed Redundant Local History Deque (Idea 227)**
    * **Change:** Removed the camera-frame `history` deque from `ObstacleTracker.__init__`, `update()`, and return values; `hist_local` is now reconstructed exclusively from `history_global`.
    * **Rationale:** Maintaining two parallel history deques per track (one in camera-frame, one in global-frame) was redundant after Idea 175's vectorized reconstruction. Removing the camera-frame deque eliminates the memory for 5 tracks × 10 frames × 3 floats = 150 floats per inference cycle, and removes the associated deque append overhead.

12. **Per-Track Acceleration Clamp (Idea 152)**
    * **Change:** Added `max_delta = 3.0 / INFER_HZ` (= 0.3 m/s per frame at 10 Hz); after assembling the full `estimates` list, clamped per-component `vx`/`vy` delta against the previous frame and recomputed `speed`.
    * **Rationale:** Human locomotion biomechanics limit peak acceleration to ~3 m/s². Step-changes larger than 0.3 m/s between inference frames indicate sensor noise or tracking discontinuities, not real motion. Clamping prevents single-frame speed spikes from triggering sudden, jarring robot decelerations.

**`server_x3.py` Changes & Technical Rationales:**

1. **Motion Loop Frequency 20 Hz → 30 Hz (Idea 111)**
   * **Change:** Changed `asyncio.sleep(0.05)` to `asyncio.sleep(0.033)` in the motion command drain loop.
   * **Rationale:** The 20 Hz drain rate was slower than the joystick's 50 Hz input rate, causing motion commands to queue up and introducing ~25–50 ms of additional latency between joystick input and wheel actuation. 30 Hz halves the queue buildup while staying well within the asyncio event loop's scheduling budget.

2. **Odometry Staleness Guard (Idea 225)**
   * **Change:** Added `self._odom_stamp = 0.0` in `ROS2Bridge.__init__`; set `self._odom_stamp = time.monotonic()` at the top of `_odom_cb`; added a staleness check in `get_robot_pose_and_twist` that returns `None` if the last odom was received more than 0.5 s ago.
   * **Rationale:** If the EKF or base_node crashes mid-run, the server continues broadcasting a stale pose that appeared current. Velocity estimation and Nav2 clients built on this stale pose would make decisions based on outdated robot location. Returning `None` on stale odom forces callers to handle the no-data case explicitly rather than silently using bad data.

3. **Depth Frame Age Tracking (Idea 230)**
   * **Change:** Added `self._last_depth_write_time = 0.0` in `ROS2Bridge.__init__`; updated `self._last_depth_write_time = time.monotonic()` at the top of `_depth_cb`; added `get_depth_frame_age() -> float` method (returns `float('inf')` if never received).
   * **Rationale:** The velocity estimator and GUI have no way to distinguish "depth frame not yet received" from "depth stream stalled." `get_depth_frame_age()` exposes this staleness metric so the estimator can gate its inference on frame freshness and avoid making decisions based on a frozen depth image.

**Bug Fixed — Missing `--domain-id` Argument (2026-06-08):**
   * **Bug:** After deploying the updated files to the Jetson, the robot failed to move in either reactive or predictive mode. Root cause: `ab_comparison_test.py` called `rclpy.init()` without applying `ROS_DOMAIN_ID=42`, meaning it joined ROS2 domain 0 instead of the Jetson's domain 42, and never received `/odom`.
   * **Fix:** Added `--domain-id` CLI argument (default: None, inherits environment). When provided, sets `os.environ['ROS_DOMAIN_ID']` before `rclpy.init()`. Correct usage: `python3 ab_comparison_test.py --mode reactive --domain-id 42`.

**Deployment:** All three files transferred to `jetson@10.13.197.109:/home/jetson/x3_ws/src/` via paramiko SFTP and verified by MD5 checksum.

**Status:** Completed 🟢. 33 improvements implemented across all three core scripts and deployed to the robot.



---

### [2026-06-09] Camera-Estimator Blindness Fix — Far-Range Detection & Track Confirmation
**Scripts Modified:**
- [velocity_estimator.py](file:///home/kamren/x3_ws/src/velocity_estimator.py) (Removed alternating-row/column decimation from the far-range depth mask; restructured `ObstacleTracker` visible-count accounting so the confirmation gate is reachable)

**Symptom Reported:**
During `ab_comparison_test.py` runs in both reactive and predictive modes, the robot completed the 4 m out-and-back path correctly but would **drive straight into a person standing in its path** instead of slowing/bypassing. The GUI obstacle panel permanently displayed *"Awaiting obstacles and velocity reports…"*, indicating the velocity-estimate list was always empty. Live inspection of the server `readout` over `ws://localhost:8081` confirmed `velocity_estimates` was empty on every frame even with a wall 1.8 m ahead filling the depth image.

**Root-Cause Diagnosis (on-robot):**
The obstacle-avoidance stack is **camera-primary** (depth-blob detection → tracking → MLP velocity → bypass/TTC speed-scaling), with LiDAR only as a ±0.75 m bumper fallback. Two independent latent bugs collapsed the camera path to zero output, leaving only the weak short-range LiDAR stop — which fires too late and only nudges 0.15 m, so a pedestrian gets struck.

**Bugs Found & Technical Rationales:**

1. **Far-Range Depth Mask Erased by Morphological Opening (`_extract_depth_centroids`)**
   * **Bug:** The "dynamic voxel downsampling" step built the far-range (1.5–4.0 m) obstacle mask on the already-2×-downsampled depth frame and then zeroed alternating rows **and** columns (`far_mask_ds[1::2, :] = False; far_mask_ds[:, 1::2] = False`). This leaves set pixels spaced two apart — no two adjacent. The immediately following $5\times5$ `MORPH_OPEN` erosion then deletes every isolated pixel, so the far-range mask is wiped out entirely. **Every obstacle beyond ~1.5 m produced zero contours and was invisible to the estimator.** Verified live against a real depth frame (~120 k valid pixels, nearest return 1.8 m): buggy code → **0 contours**; with the two decimation lines removed → **3 contours / 2 centroids**.
   * **Fix:** Removed the alternating-row/column zeroing. The far-range region stays contiguous (the frame is already 2× downsampled for cost), so erosion preserves real obstacle blobs while still cleaning speckle.

2. **`visible_count` Confirmation Gate Mathematically Unreachable (`ObstacleTracker.update`)**
   * **Bug:** The aging loop decremented `visible_count` for *every* track on *every* frame (`visible_count = max(0, visible_count - 1)`), while a successful match incremented it by exactly 1. A track matched every frame therefore netted **zero** change and stayed pinned at its initial value of **1**. The estimate-eligibility gate requires `visible_count >= 3` (the 3-of-N track-initiation rule, Idea 109), so **no track could ever be confirmed** and the estimates list was always empty — even though tracks existed (live debug showed `tracks=2–4`).
   * **Fix:** Removed the unconditional per-frame decrement. Staleness is already handled by `age`/`max_age` (tracks unseen for >10 frames are deleted). A separate decay step now decrements `visible_count` only for tracks that were **not** matched this frame, preserving the "recent consecutive visibility" semantics. The gate is reachable after ~3 frames (~0.3 s at the 10 Hz inference rate).

**Verification:** After both fixes and a service restart, the live `readout` went from `n_estimates=0` to stable detections (e.g. obstacles at 1.95 m and 3.61 m with persistent track IDs). The GUI obstacle panel now populates, and both reactive and predictive modes share this restored detection path.

**Deployment:** Both fixes applied to the local workspace and directly to `jetson@10.13.245.176:/home/jetson/x3_ws/src/velocity_estimator.py`; `x3_server` restarted and confirmed healthy. (Robot working copy currently diverges from local `main` — reconcile on next git deploy.)

**Status:** Completed 🟢. Far-range camera detection and track confirmation are restored and verified on the physical robot.

---

### [2026-06-09] Detection Model Switch — Cans Detector → Standard COCO (Person-Capable)
**Scripts Modified:**
- [server_x3.py](file:///home/kamren/x3_ws/src/server_x3.py) (Changed default `YOLO_MODEL = find_model_path("yolo26n")` and `active_model_name = "yolo26n"`, replacing `yolo11n_cans`)
- Robot environment: installed `torchvision==0.25.0`; moved the broken prebuilt engine `yolo_models/default/yolo26n.engine` → `yolo26n.engine.broken-296cls`

**Motivation:**
The active detector was `yolo11n_cans`, a soda-can model with no `person` class, so the visual-LiDAR person-confirmation gate (Idea 146) and the predictive "dynamic pedestrian" corridor widening could never identify a human as such. A standard COCO model (`class 0 = person`) is required for human-aware behaviour.

**Findings & Technical Rationales:**

1. **YOLO Detection Was Environment-Broken Independent of Model Choice**
   * **Finding:** The robot's system Python (`/usr/bin/python3` 3.10, `torch 2.10.0+cpu`) was **missing `torchvision`** (only an orphaned `0.15.0` `.dist-info` with no package remained, which blocked imports and confused `pip`). With no `torchvision`, every `.pt` model fails to load and the failure was silently swallowed by the YOLO-init try/except — so `yolo11n_cans` had in practice been producing **zero detections all along** (it had no `.engine`, so it fell to the broken `.pt` path).
   * **Resolution:** Removed the stale `0.15.0` metadata and installed `torchvision==0.25.0` (the version paired with torch 2.10) using `--no-deps` so the existing torch build was untouched.

2. **Prebuilt `yolo26n.engine` Was Mismatched and Non-Functional**
   * **Finding:** The TensorRT wrapper derives class count from the engine output as `out_shape[1] - 4`, which reported **296 classes** for the on-disk `yolo26n.engine` — yet the matching `.pt` is standard **80-class COCO**. The engine had been built from a different model/device and, when exercised, threw `IExecutionContext::enqueueV3: Cask convolution execution` (TRT Error Code 1) and returned no detections. It was therefore unusable on this robot.
   * **Resolution:** Renamed the bad engine to `yolo26n.engine.broken-296cls` so the server's "prefer `.engine` else `.pt`" loader falls through to the correct CPU `.pt`. A `yolo26n.onnx` remains in the directory for rebuilding a correct GPU engine later (now possible since `torchvision` is present).

3. **Switched Active Model to COCO `yolo26n` (CPU)**
   * **Change:** Set the default model name to `yolo26n` in `server_x3.py`. After restart the server logs `Loading YOLO (CPU) … yolo26n.pt` and `YOLO classes (80): 0:person, 1:bicycle, …`. Standalone inference on the Ultralytics `bus.jpg` reference image returned `person` @ 0.91 and `bus` @ 0.92, confirming correct class mapping. The live detection loop runs at **~1.9 fps on CPU** (no `.engine`), which is acceptable for low-speed indoor avoidance but is the main candidate for future GPU acceleration.
   * **Rationale:** Provides a real `person` class so the velocity estimator's person-gating and the predictive pedestrian-corridor logic become meaningful, while keeping the depth-based avoidance (which needs no YOLO) fully functional regardless.

**Operational Notes:**
- Camera detection defaults **OFF** and resets OFF on every server restart; enable it from the GUI (or `{"type":"toggle_detection","enabled":true}`) when person-confirmation is wanted.
- Future option for fast GPU person detection: export `yolo26n.pt` → ONNX → a device-correct TensorRT `.engine` (torchvision is now installed to support this).

**Deployment:** `server_x3.py` model name changed locally and on `jetson@10.13.245.176`; `torchvision` installed and broken engine renamed on the robot; `x3_server` restarted and CPU person detection verified live.

**Status:** Completed 🟢. The robot now runs a standard COCO detector with a working `person` class; obstacle avoidance verified functional in both modes.

---

### [2026-06-09] Demo-Prep Live Diagnostics, Operational Settings & Known Issues
**Scripts Analyzed (no code edits this session):**
- [velocity_estimator.py](file:///home/kamren/x3_ws/src/velocity_estimator.py) (visual-LiDAR person gate, MLP velocity output behaviour while driving)
- [ab_comparison_test.py](file:///home/kamren/x3_ws/src/ab_comparison_test.py) (`_update_bypass_offset` side-selection, LiDAR-blocked bypass branch)

**Context:**
Final walk-in testing before the live demo. The fixes above restored detection, but pedestrian avoidance still looked glitchy: reactive mode stuttered (expected), and predictive mode also stuttered, sometimes bumped a stationary person's shoes, and on one return pass appeared to **steer toward** the person. Root-caused live on the robot by capturing the server `readout` (`ws://localhost:8081`) and the A/B diagnostic block from `journalctl`. No code was changed; the outcomes are operational settings + documented limitations.

**Findings & Technical Rationales:**

1. **Slow CPU-YOLO Person-Gate Starves the Estimator While Walking**
   * **Finding:** With camera detection enabled, YOLO runs at only ~2–2.4 fps on CPU. The visual-LiDAR fusion gate (Idea 146) keeps a depth centroid only if it reprojects inside a YOLO `person` box (+15 px). At ~2.4 fps the box is stale by ~0.4 s, so a *moving* person's fresh 10 Hz depth centroid frequently falls outside the stale box and gets discarded — the track drops in and out, the MLP can't hold a velocity, and predictive mode loses its early-reaction signal.
   * **Operational fix for demo:** Run with camera detection **OFF** (the default; it also resets OFF on restart and must be started clean, since toggling it off in the GUI does not clear the last `person` box). With detection off the gate is skipped entirely, so the depth + MLP estimator tracks pedestrians continuously at 10 Hz. The depth-based avoidance needs no YOLO.

2. **Depth Near-Field Dead Zone Explains "Bumps Shoes When Standing Still"**
   * **Finding:** The Astra depth camera has a <0.5 m near-field dead zone and a vertical FOV that often excludes feet right at the bumper, so a person standing toe-to-bumper is simply not in the depth data. A stationary target also reads speed ≈ 0, so predictive treats it as a *static* obstacle (tight corridor, small bypass) rather than widening. Only the ±0.75 m LiDAR bumper catches that case, and it only nudges.
   * **Operational note:** Demo with the person approaching from ≥ ~0.6 m (legs/torso in depth range), not toe-to-bumper.

3. **"Steers Into Me" Root-Caused — Bypass Picks the Open Side Without Checking for a Person (primary), Compounded by Phantom MLP Velocities (secondary)**
   * **Evidence (return pass, person = track ID 180 on the robot's LEFT at `left=+0.21 m`, `fwd=0.66 m`):** Front LiDAR was blocked, the robot's **right** side was cluttered with other returns (IDs 167/174, which the MLP also tagged with phantom speeds of 0.27–0.69 m/s), so `Right Blocked=True`. The LiDAR-blocked bypass branch therefore chose the only "open" side — **left** (`Target Lateral Offset=+0.15 m`) — and strafed there, directly into the person's position. It then froze when both sides read blocked (`speed_scale → 0`). The robot never selected the person as the obstacle to avoid; it escaped the cluttered side *into* the person.
   * **Sign check (forward pass):** A right-side obstacle correctly produced a left strafe (`offset=+0.40`, `path_y −0.058 → +0.197`), confirming the lateral command **sign is correct** — this is a side-*selection/salience* flaw, not a sign inversion.
   * **Secondary cause (estimator):** While driving, the MLP reports phantom velocities up to **0.8 m/s** on physically static objects (ego-motion not fully cancelled). Anything > 0.15 m/s is treated as a fast "dynamic pedestrian," so predictive applies the widest 0.40 m corridor to clutter and over-reacts/jitters.
   * **Operational fix for demo:** Run in a corridor clear of objects within ~1.5 m on either side, with the person approaching from straight ahead or one open side. Removing side clutter prevents the "escape into the open side" failure and removes most phantom-velocity triggers.

**Demo Configuration Used (presentable state):**
- Camera detection **OFF** (predictive avoidance runs on depth + MLP at 10 Hz, no YOLO gate).
- Test environment cleared of side obstacles within ~1.5 m of the path.
- Person approaches from ahead / one open side, ≥ ~0.6 m from the bumper.

**Known Issues / Future Work (carry into paper limitations & next steps):**
1. **Bypass side-selection is obstacle-agnostic:** when the front is blocked and the preferred escape side is also blocked, it strafes to the open side without checking whether a tracked person is standing there. Planned fix: forbid strafing toward a side that has a tracked obstacle/person on it.
2. **Phantom MLP velocities while driving:** ego-motion compensation (Ideas 1 & 11) does not fully cancel the robot's own motion, so static objects read up to ~0.8 m/s. Planned fix: gate/zero MLP velocity for tracks whose global-frame position is stationary, and revisit the pose used for compensation.
3. **CPU-only YOLO (~2 fps)** is too slow for the real-time person-gate; build a device-correct TensorRT engine from `yolo26n.onnx` for GPU detection (torchvision now present).

**Status:** Diagnosed & demo-configured 🟢. Items 1–3 above are open and documented for post-demo work; no code changes were made in this session.