# EE 244 Final Project - Progress & Matrix

This document tracks the implementation progress, technical challenges encountered, and the reasoning behind each major change in the project. It also serves as the master tracking matrix for weekly deliverables.

## Project Tracking Matrix

| Week | Phase | Goal / Deliverable | Status |
| :--- | :--- | :--- | :--- |
| **Weeks 1-2** | **Data Preparation** | Preprocess THÖR-MAGNI dataset. Extract synchronized triplets at 10Hz. | Completed 🟢 |
| **Week 3** | **Initial Training** | Train and validate on THÖR-MAGNI. Hold out 20% sequences. Establish MAE baseline against Kalman filter. | Completed 🟢 |
| **Week 4** | **Domain Adaptation** | Collect 30-min domain adaptation set in deployment environment. Process rosbags offline into X/y windows matching THÖR-MAGNI format. | In Progress 🟡 |
| **Weeks 5-6** | **Fine-Tuning** | Fine-tune model on collected data (learning rate ≤ 1e-4, freezing early layers) to bridge sensor gap. | Not Started ⚪ |
| **Week 7** | **Demo Prep** | Focus on A/B comparison (MPPI with/without velocity estimates). Visualize in RViz2 with velocity arrows. | Not Started ⚪ |

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

**Status:** Infrastructure ready. Pending: physical classroom data collection session on Jetson.
