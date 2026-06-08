# Project Improvement Ideas ROI Analysis

This document records the ROI rankings and evaluation metrics for the enhancement ideas logged in `improvement_ideas.md` for the predictive planning and local velocity estimation project.

## Last Updated
2026-06-08: Added 22 ideas (199–220) across tiers. 13 new High-ROI, 9 new Medium-ROI entries added.

---

## 1. Already Implemented Ideas
The following **32 ideas** have already been completed and integrated:
*   **Idea 1**: Ego-Motion Compensation in `VelocityEstimator`
*   **Idea 2**: Live ROS2 Topic Publishing for Nav2/MPPI Integration
*   **Idea 3**: PD-based Closed-Loop Heading Control in Calibration Nodes
*   **Idea 9**: RViz2 Spatio-Temporal Trajectory & Velocity Vector Markers
*   **Idea 11**: Encoders-IMU Slip Compensation using EKF Velocity Feedback
*   **Idea 13**: Dynamic Proximity & Time-to-Collision (TTC) Speed Scaling
*   **Idea 17**: Subprocess Lifecycle Management and Orphan Prevention
*   **Idea 18**: Cross-Track Error Correction Using Holonomic Strafing
*   **Idea 19**: Depth-Aware 3D Centroid Reconstruction for Velocity Scaling
*   **Idea 21**: Direct Meters-Based Depth Processing to Avoid Normalization Artifacts
*   **Idea 22**: Pedestrian Clearance Distance Logging
*   **Idea 26**: Heading Drift Correction via SLAM Loop Closures in Test Run Recovery
*   **Idea 29**: Adaptive Lateral Bypass Direction Selection
*   **Idea 30**: Smoothed Lateral Velocity Profiles for Mecanum Slip Prevention
*   **Idea 31**: Dynamic Path Re-planning Timeout Scaling
*   **Idea 36**: Cropped Contour Masking for Centroid Extraction
*   **Idea 43**: Temporal Depth Filtering for Centroid Smoothing
*   **Idea 45**: Blended Diagonal Bypass Path Profiling
*   **Idea 46**: Batched MLP Inference for Multi-Track Scaling
*   **Idea 48**: Local Trajectory Translation Normalization
*   **Idea 52**: Decimated Depth Arrays for Centroid Median Calculation
*   **Idea 56**: SIMD-Optimized OpenCV inRange Thresholding
*   **Idea 63**: Robust Coordinate Clamping and Velocity Bound Constraints
*   **Idea 67**: Battery Voltage Query Throttling in ROS2 Bridge
*   **Idea 70**: Omnidirectional Direction-of-Travel Coordinate Projection
*   **Idea 71**: Nav2 Action Status Query Throttling
*   **Idea 72**: Pure NumPy Feature Normalization (Eliminating Scikit-Learn)
*   **Idea 141**: Active Lateral Centering Potential Field for Corridor Navigation
*   **Idea 143**: Dynamic Point Cloud Downsampling with Adaptive Voxel Grid Gating
*   **Idea 144**: LiDAR Scan Matching for Relative Corridor Slip Estimation
*   **Idea 145**: Zero-Copy Memory-Mapped Frame Allocation via Shared Memory IPC
*   **Idea 146**: Visual-LiDAR Geometric Depth Fusion (Sensor Fusion Gating)

---

## 2. High-ROI Tier (Rank 1 - 43)
*These enhancements require minimal development effort (simple code adjustments, caching, or standard formulas) but deliver substantial improvements in CPU/RAM usage, execution efficiency, stability, or model accuracy.*

### 1. Idea 81: Immediate Depth Downsampling for Centroid Extraction
*   **Investment:** Extremely Low. Resize the raw depth image immediately using fast slicing offsets or nearest-neighbor resizing.
*   **Return:** Extremely High. Cuts CPU processing time and memory allocations of the bottleneck contour extraction step by 75% to 90% without degrading spatial accuracy.

### 2. Idea 136: Vectorized Frame Downsampling via Slice Offsets
*   **Investment:** Extremely Low. Use NumPy slicing `[::2, ::2]` instead of calling external resize operations.
*   **Return:** High. Eliminates Python/C++ binding overhead and runs in sub-microseconds, saving CPU capacity.

### 3. Idea 121: Vectorized Centroid Global Transformation via Matrix Dot Products
*   **Investment:** Low. Convert centroids to a NumPy array and transform via matrix dot product.
*   **Return:** High. Replaces Python coordinate projection loops with native C/SIMD dot products, reducing frame time.

### 4. Idea 91: Vectorized Pairwise Distance Matrix Broadcasting
*   **Investment:** Low. Replace tracking distance checks with NumPy matrix broadcasting subtraction.
*   **Return:** High. Bypasses Python loop overhead, scaling tracks matching efficiently.

### 5. Idea 111: Serial Command Write Rate Optimization (30Hz Throttle)
*   **Investment:** Extremely Low. Change command sleep interval from 100Hz to 30Hz in `server_x3.py`.
*   **Return:** High. Reduces serial write CPU overhead by 70% and prevents serial port lock contention on the Jetson.

### 6. Idea 87: High-Performance orjson Serialization
*   **Investment:** Extremely Low. Replace `import json` with `import orjson` in server script.
*   **Return:** High. Speeds up serialization of complex nested telemetry dictionaries significantly.

### 7. Idea 108: Kinematic Stop-Trigger Gating
*   **Investment:** Low. Fast displacement checks over the last 3 frames to force zero-velocity outputs.
*   **Return:** High. Eliminates MLP sequence prediction lag when a pedestrian halts abruptly.

### 8. Idea 109: Centroid Presence Verification Filter (Track Initiation Gate)
*   **Investment:** Low. Add a candidate buffer requiring presence in 3 out of 5 frames before tracking.
*   **Return:** High. Suppresses transient specular/dust depth ghosts from triggering false braking maneuvers.

### 9. Idea 110: Dynamic Corridor Width Clearance Scaling
*   **Investment:** Low. Scale bypass offset target bounds dynamically using left/right lidar scan clearances.
*   **Return:** High. Prevents the robot from commanding lateral strafes into walls in narrow hallway spaces.

### 10. Idea 116: Pre-Allocated PyTorch Input Tensor Reuse
*   **Investment:** Low. Reuse a pre-allocated input tensor shape rather than instantiating PyTorch Tensors at 10Hz.
*   **Return:** High. Reduces garbage collector heap allocations and memory bandwidth overhead.

### 11. Idea 122: Dynamic Path Deceleration via Predictive TTC Hysteresis
*   **Investment:** Low. Apply an EMA smoothing/hysteresis check to the calculated speed scale factor.
*   **Return:** High. Smooths out dynamic velocity scaling and prevents robot speed chattering and wheel slip.

### 12. Idea 80: Rear-Sector LiDAR Protection for Backing Recovery
*   **Investment:** Low. Add a check on angles 135–225 in `ab_comparison_test.py` during backing.
*   **Return:** Very High. Prevents backing collision accidents in the blind spot during active timeout recoveries.

### 13. Idea 137: Pre-Compiled PyTorch Modules via JIT Tracing (Trace-Optimization)
*   **Investment:** Low. Run tracing once at startup with a static tensor placeholder.
*   **Return:** Medium-High. Speeds up PyTorch model execution by enabling compiler fusion optimization.

### 14. Idea 106: Downscaled Depth Visualization Encoding
*   **Investment:** Low. Resize depth frame to 320x240 before running JPEG compression.
*   **Return:** High. Cuts server JPEG encoding CPU usage by 75% for GUI preview feeds.

### 15. Idea 117: Non-Blocking WebSockets Payload Compression
*   **Investment:** Low-Medium. Disable deflate or run compression in background thread executors.
*   **Return:** High. Prevents event loop lockups, securing real-time low-latency motor commands.

### 16. Idea 188: LaserScan Range Conversion to NumPy Array
*   **Investment:** Extremely Low. One-line change: `np.array(msg.ranges, dtype=np.float32)` in `_scan_cb`.
*   **Return:** Extremely High. Unlocks vectorized NumPy ops throughout the entire LiDAR processing pipeline in `_update_bypass_offset` and ICP, replacing Python element-by-element loops with C-level SIMD operations.

### 17. Idea 187: Precomputed LiDAR Angle Array Cache
*   **Investment:** Extremely Low. Cache `normalize_angle(angle_min + i * angle_increment + math.pi)` as a NumPy array, recompute only when scan params change.
*   **Return:** Extremely High. Eliminates per-beam trigonometry in the Python loop inside `_update_bypass_offset`, enabling all sector-mask computations to run as single vectorized boolean operations.

### 18. Idea 198: ICP Point Cloud Uniform Subsampling for CPU Efficiency
*   **Investment:** Extremely Low. Add `pts = pts[::max(1, len(pts)//80)]` before the ICP outer loop in `_align_scans_icp`.
*   **Return:** Extremely High. Reduces the O(M×N) inner distance matrix from ~360×360 to ~80×80 (20× fewer elements per iteration) with no meaningful accuracy degradation on planar corridor walls.

### 19. Idea 138: Vectorized Spatial Grid Binning for LiDAR Points
*   **Investment:** Low. Precompute angular sector index masks at init; apply them with NumPy boolean indexing each tick.
*   **Return:** High. Replaces the Python for-loop over all 360 range beams in `_update_bypass_offset` with a constant-time NumPy mask operation, cutting bypass analysis overhead.

### 20. Idea 189: Adaptive ICP Convergence Criterion with Early Exit
*   **Investment:** Low. Add `if abs(dx_corr) + abs(dy_corr) + abs(dtheta_corr) < 1e-4: break` after each ICP iteration.
*   **Return:** High. For stationary or slow segments, ICP converges in 1 iteration; the change saves 1–2 full O(N²) distance-matrix computations per scan invocation at 20 Hz.

### 21. Idea 197: EMA-Smoothed Wall Clearance Values for APF Jitter Prevention
*   **Investment:** Extremely Low. Add two EMA state variables and apply `smoothed = 0.7*new + 0.3*prev` before computing `f_rep_left` and `f_rep_right` in `_update_bypass_offset`.
*   **Return:** High. Eliminates single-frame LiDAR spike artifacts that cause the APF force to jerk lateral commands, preventing wheel-slip chattering at zero additional CPU cost.

### 22. Idea 194: ICP Correction Delta Bound Clamping for Long-Run Stability
*   **Investment:** Low. Add a rejection check in `_scan_cb` before applying ICP-computed corrections to `corrected_x/y/yaw`.
*   **Return:** High. Prevents one degenerate scan pair from injecting a large spurious correction into the drift reference pose, which would corrupt all subsequent path-tracking distance calculations for the rest of the run.

### 23. Idea 190: ICP Scan Match Inlier Quality Gate
*   **Investment:** Low. After the last ICP iteration, check `if np.sum(valid) < 0.20 * len(Q_trans): use raw odometry`.
*   **Return:** High. In open doorways or sparse scans, the 3-iteration ICP produces noise; the gate prevents bad alignments from reaching `corrected_x/y/yaw` while adding only a single comparison per scan callback.

### 24. Idea 152: Physically Constrained MLP Output Gating
*   **Investment:** Low. Clamp the predicted velocity change between consecutive frames to a maximum human acceleration of 3.0 m/s² in `_inference_loop`.
*   **Return:** High. Prevents spike predictions caused by noisy inputs or contour discontinuities from triggering erratic TTC braking without requiring any model retraining.

### 25. Idea 166: Dynamic Depth Segment Size Thresholding Based on Range
*   **Investment:** Low. Replace constant `MIN_BLOB_AREA / 4.0` check with `area < base_area * (Z_ref / Z)**2` in `_extract_depth_centroids`.
*   **Return:** High. Prevents far-away pedestrians (at 3.0–4.0 m) from being filtered out by the fixed area threshold, resolving a systematic detection failure that causes track loss at range.

### 26. Idea 160: Fast-Path Gating for Empty Track States
*   **Investment:** Low. Add a single `if np.all(np.isnan(depth_frame) | (depth_frame < 0.5) | (depth_frame > 4.0)): skip` at the top of `_inference_loop`.
*   **Return:** High. Eliminates all contour finding, tracker association, and PyTorch inference overhead when the corridor ahead is clear, which applies to most of the free-driving segments.

### 27. Idea 171: Vectorized LiDAR-based Yaw-Drift Corrector
*   **Investment:** Low. Find `np.argmin(side_ranges)` in left/right sectors to compute perpendicular wall angle; derive yaw drift as angular deviation from 90°.
*   **Return:** High. Provides an O(N) direct yaw-drift correction from corridor wall geometry, complementing ICP and usable as a lightweight check between full ICP scan updates.

### 28. Idea 175: Vectorized 2D Rotation via NumPy Matrix Dot Products
*   **Investment:** Low. Stack `history_global` coordinates into shape (T, 2) and compute `hist_local = (hist_global - robot_xy) @ R.T` in one call in `_inference_loop`.
*   **Return:** High. Replaces the Python per-frame loop reconstructing `hist_local` for each eligible track with a single matrix multiply, eliminating T×N Python interpreter iterations per inference step.

### 29. Idea 155: Vectorized LiDAR Point Clustering via Range Gradient Masking
*   **Investment:** Low. Compute `gaps = np.abs(np.diff(ranges)) > eps` and use `np.where` to slice connected segments as clusters.
*   **Return:** Medium-High. Provides an O(N) clustering alternative to DBSCAN for extracting obstacle centroids directly from the LiDAR scan, useful for backup tracking or fusion gating.

### 30. Idea 150: Fast 1D Angular-Index Search for LiDAR Scan Matching
*   **Investment:** Low. Convert point clouds to polar form; for each current point find the nearest previous-scan range via direct angular bin lookup instead of building the full 2D distance matrix.
*   **Return:** Medium-High. Replaces the O(M×N) inner ICP correspondence loop with O(M) direct lookups, reducing scan-matching CPU overhead significantly especially in wider scans.

### 31. Idea 199: Repeat-Mode `prev_scan_points` Reset to Prevent Cross-Run ICP Corruption
*   **Investment:** Extremely Low. Add `self.prev_scan_points = []` alongside the existing `prev_odom_pose` reset in the `ROTATE_HOME` repeat-mode block.
*   **Return:** Extremely High. Eliminates the first-scan ICP corruption artifact on every repeated run with a single one-line change; without it, large spurious corrections are injected into `corrected_x/y/yaw` before the robot has moved.

### 32. Idea 200: Non-Blocking Recovery Backing via Timer-Based State
*   **Investment:** Low. Replace the 15×`time.sleep(0.1)` loop in the recovery block with an `_is_backing` boolean and `_backing_start` timestamp checked in the normal 20 Hz control path.
*   **Return:** Very High. Eliminates 1.5 s of ROS2 callback starvation during backing, restoring LiDAR safety checks and ICP updates at precisely the moment when rear collision risk is highest.

### 33. Idea 201: `visible_count` Decay on Unmatched Track Frames
*   **Investment:** Extremely Low. Add one line `track['visible_count'] = max(0, track['visible_count'] - 1)` in the existing track-aging loop in `ObstacleTracker.update()`.
*   **Return:** High. Prevents permanently eligible ghost tracks from sustaining TTC scaling contributions through arbitrarily long occlusion gaps, eliminating spurious braking from stale observations.

### 34. Idea 202: Division Pre-Inversion for Faster Feature Normalization
*   **Investment:** Extremely Low. Pre-compute `self.scaler_X_inv_scale = (1.0 / self.scaler_X_scale).astype(np.float32)` in `_load_model()` and replace the per-inference division with multiplication.
*   **Return:** High. ARM NEON float32 division is ~2× slower than multiplication; the change applies to the (N, 40) normalization matrix executed 10 times per second with zero numerical difference.

### 35. Idea 203: Multi-Frame Confirmation Counter for `_front_is_continuous_wall`
*   **Investment:** Extremely Low. Add a `_wall_confirm_count` integer and expose the flag only when count ≥ 2, requiring 100 ms of consistency before suppressing bypass or triggering early stop.
*   **Return:** High. Prevents single-frame scan glitches (smooth doorframes, specular returns) from incorrectly blocking camera-based bypass initiation or triggering the static-wall early stop.

### 36. Idea 204: Per-Track Distance Gate Before MLP Inference to Skip Far Targets
*   **Investment:** Extremely Low. Add an early-continue guard `if track['centroid'][2] > PROXIMITY_THRESHOLD: continue` before appending to `features_list` in `_inference_loop`.
*   **Return:** High. Eliminates inference compute, feature assembly, and normalization for all tracks beyond 1.8 m that contribute zero to speed scaling, directly reducing batch size and forward-pass latency in crowded long-range scenes.

### 37. Idea 205: Cross-Track Error and Commanded Velocity Columns in RunLogger
*   **Investment:** Extremely Low. Add `current_path_y`, `last_vx_cmd`, `last_vy_cmd`, `vy_rep` as additional fields to `_maybe_log()` in `DRIVE_TO_B` and `DRIVE_TO_A` states.
*   **Return:** High. These are the primary metrics for evaluating A/B path-following quality and APF centering effectiveness; their absence from the CSV requires manual reconstruction from secondary columns during post-analysis.

### 38. Idea 207: Settle-State Stop Command Deduplication via Published-Flag
*   **Investment:** Extremely Low. Add `self._motor_is_stopped` flag: `_stop_robot()` publishes only when flag is False, then sets it True; any non-zero command resets it to False.
*   **Return:** High. Reduces `/cmd_vel` DDS publish rate from 60 Hz to a single burst per settle-state entry, cutting DDS middleware overhead and Mcnamu driver processing load during all three settle states.

### 39. Idea 208: Adaptive ICP Iteration Count Based on Commanded Speed
*   **Investment:** Low. Replace `range(3)` in `_align_scans_icp` with `range(1 if abs(self.last_vx_cmd) < 0.05 else 3)`.
*   **Return:** High. Reduces ICP CPU cost by two-thirds during near-waypoint deceleration and settle-state entries, which constitute a significant fraction of run time, while preserving full 3-iteration accuracy at high traverse speeds.

### 40. Idea 209: Cache `start_yaw` Trigonometry Constants at Segment Transitions
*   **Investment:** Extremely Low. Pre-cache `self._cos_start_yaw = math.cos(self.start_yaw)` and `self._sin_start_yaw = math.sin(self.start_yaw)` at the SETTLE→DRIVE transition once.
*   **Return:** High. Eliminates 4 transcendental function calls per 20 Hz tick (80 FPU operations per second) in the distance and cross-track error projection calculations in both `DRIVE_TO_B` and `DRIVE_TO_A`, replacing them with free float variable reads.

### 41. Idea 211: Single `_estimates` Snapshot Per Control Loop Tick
*   **Investment:** Extremely Low. Pre-fetch `self._latest_estimates` once at the top of the drive-state block and pass the snapshot to both `_get_speed_scaling()` and `_update_bypass_offset()`.
*   **Return:** High. Eliminates one redundant lock acquisition and one `list()` copy per drive iteration (from 2 to 1 per tick), halving lock contention on the estimates path at 20 Hz.

### 42. Idea 212: Store `prev_scan_points` Directly as Pre-Converted NumPy Array
*   **Investment:** Extremely Low. Change `self.prev_scan_points = curr_pts` (Python list) to `self.prev_scan_points = np.array(curr_pts, dtype=np.float32)` in `_scan_cb`.
*   **Return:** High. Removes the `P = np.array(prev_pts, dtype=np.float32)` conversion from the ICP hot path called at 20 Hz, eliminating a per-call allocation and copy that currently runs inside the performance-critical scan-matching function.

### 43. Idea 213: Vectorized Continuous Wall Edge Detection via `np.diff` on Front Sector
*   **Investment:** Extremely Low. Replace the Python per-beam loop computing `front_has_edges` with `diffs = np.abs(np.diff(front_ranges)); front_has_edges = bool(np.any(diffs > 0.4))` after the front-sector NumPy slice.
*   **Return:** High. Replaces hundreds of Python interpreter steps per tick with a single C-level SIMD call, keeping the wall detection logic identical while running in microseconds; depends on Idea 188 (LaserScan NumPy conversion) being implemented first.

---

## 3. Medium-ROI Tier (Rank 44 - 111)
*These enhancements offer good performance or navigation benefits, but require moderate coding effort, extra message handling, or minor model changes.*

*   **16. Idea 38: Velocity-Projected Constant Velocity Tracker Association** (Medium-High ROI)
*   **17. Idea 35: Continuous, Gap-Based Lateral Bypass Offset Calculation** (Medium-High ROI)
*   **18. Idea 42: Shell Sourcing Elimination for ROS2 Subprocesses** (Medium ROI)
*   **19. Idea 51: Client-Side Drawing Overlay Delegation** (Medium ROI)
*   **20. Idea 20: Temporal Consistency and Interpolation in Track History** (Medium ROI)
*   **21. Idea 65: Forward-Projected Path Collision Corridors** (Medium ROI)
*   **22. Idea 40: Swept-Volume Rotational Safety Fields** (Medium ROI)
*   **23. Idea 66: ONNX Runtime Conversion for PyTorch-Free Execution** (Medium ROI)
*   **24. Idea 50: Footprint-Swept Corridor Expansion for Holonomic Path Protection** (Medium ROI)
*   **25. Idea 16: Bi-Directional Telemetry and Progress Reporting for A/B Runs** (Medium ROI)
*   **26. Idea 34: Distance-Adaptive Area Thresholding for Near-Field Detections** (Medium ROI)
*   **27. Idea 57: Binary Serialization of Depth Frames via WebSockets** (Medium ROI)
*   **28. Idea 68: Kinematic Back-Propagation for Track Initialization Padding** (Medium ROI)
*   **29. Idea 53: Bipartite Matching for Tracker Centroid Association** (Medium ROI)
*   **30. Idea 54: Morphological Flood-Fill Hole Correction for Specular Dropouts** (Medium ROI)
*   **31. Idea 77: Connected Components with Stats for Centroid Extraction** (Medium ROI)
*   **32. Idea 44: Vertical Centroid Fusion for Partially Occluded Pedestrians** (Medium ROI)
*   **33. Idea 24: SLAM-Based Odometry Drift Logging for Ground-Truth Evaluation** (Medium ROI)
*   **34. Idea 32: Damping Gain Scheduling for Lateral Alignment** (Medium ROI)
*   **35. Idea 58: Kinematic Derivative Enrichment for MLP Input Vectors** (Medium ROI - Retraining required)
*   **36. Idea 14: Temporal Jitter Mitigation in A/B Run Logs via Synced Message Buffering** (Medium-Low ROI)
*   **37. Idea 15: Path Yielding and Recovery States for Blocked Trajectories** (Medium-Low ROI)
*   **38. Idea 33: Dynamic Temporal Scaling of Centroid Displacements** (Medium-Low ROI)
*   **39. Idea 86: Adaptive Duty-Cycle Throttling in Estimator Idle States** (Medium-High ROI)
*   **40. Idea 92: Pre-Allocated LUT Map Pixel Mapping** (Medium-High ROI)
*   **41. Idea 96: Precomputed Pinhole Projection Grid (Projection LUT)** (Medium-High ROI)
*   **42. Idea 97: Decoupled High/Low Frequency Telemetry Split** (Medium-High ROI)
*   **43. Idea 101: Pre-Allocated NumPy Ring Buffers for Coordinate History** (Medium-High ROI)
*   **44. Idea 102: Event-Driven Camera-Synced Broadcast Loop** (Medium-High ROI)
*   **45. Idea 112: Single-Precision float32 Scale Arithmetic** (Medium-High ROI)
*   **46. Idea 120: Target-Centric Kalman Filter Gate for WebSocket Telemetry Bandwidth Reduction** (Medium-High ROI)
*   **47. Idea 127: WebSocket Frame Compression Bypass for Small Payloads** (Medium-High ROI)
*   **48. Idea 78: Kinematic Acceleration-Limiting Output Filter** (Medium ROI)
*   **49. Idea 79: Semantic Gating with YOLO Bounding Boxes** (Medium ROI)
*   **50. Idea 82: Dynamic Subscription Lifecycle Management for Idle Power Savings** (Medium ROI)
*   **51. Idea 83: Hardware-Triggered Pose-Timestamp Interpolation** (Medium ROI)
*   **52. Idea 84: Edge-Proximity Confidence Weighting for FOV Transitions** (Medium ROI)
*   **53. Idea 85: Predictive Dynamic Corridor Shifting** (Medium-High ROI)
*   **54. Idea 88: Heading-Aligned Path Rotation for Trajectory Invariance** (Medium ROI)
*   **55. Idea 93: MLP-velocity dead reckoning for occluded tracks** (Medium ROI)
*   **56. Idea 95: Dynamic side-clearance potential fields** (Medium-High ROI)
*   **57. Idea 99: Depth Histogram Peak-Slicing for Dynamic Segmenting** (Medium ROI)
*   **58. Idea 104: Temporal Inter-Frame Depth Differencing (Motion Masking)** (Medium ROI)
*   **59. Idea 105: Active Center-Point Drift Correction during Rotation** (Medium-High ROI)
*   **60. Idea 107: Double Exponential Smoothing (Holt's Linear Trend)** (Medium ROI)
*   **61. Idea 115: Live LiDAR-Wall Relative Alignment at Waypoint A** (Medium-High ROI)
*   **62. Idea 118: Spatio-Temporal Cluster Merging for Multi-Scale Blob Extraction** (Medium-High ROI)
*   **63. Idea 119: Adaptive Lidar-Based Dynamic Scan Angle Filtering** (Medium-High ROI)
*   **64. Idea 123: Depth-Range-Adaptive Morphological Filtering** (Medium ROI)
*   **65. Idea 124: IMU-Aided Visual Odometry Ego-Motion Projection** (Medium-High ROI)
*   **66. Idea 125: Dynamic Potential Fields for Path Clearance Margins in Narrow Corridor Navigation** (Medium-High ROI)
*   **67. Idea 126: Vectorized Depth Slicing using Bounding Box Coordinates** (Medium ROI)
*   **68. Idea 128: Auto-Calibrating Depth Threshold Offset via Ambient Lighting Reference** (Medium ROI)
*   **69. Idea 129: Multi-Scale Temporal Windowing for Motion Feature Engineering** (Medium ROI)
*   **70. Idea 130: Predictive Corridor Yaw Alignment during Bypass Strafing** (Medium-High ROI)
*   **71. Idea 131: Memory-Mapped Shared Ring Buffer for Web Server IPC** (Medium ROI)
*   **72. Idea 133: Dynamic Scaling of DBSCAN Search Radius (Eps) for LiDAR Points** (Medium ROI)
*   **73. Idea 134: IMU-Based Visual Centroid Stabilization under Pitch/Roll Vibrations** (Medium ROI)
*   **74. Idea 135: Trajectory-Aware Predictive Safety Slowdown Gating** (Medium-High ROI)
*   **75. Idea 147: Scan-Match Corrected Ego-Motion Compensation for Tracker History** (Medium-High ROI) — Expose `corrected_x/y/yaw` from `ab_comparison_test.py` to `velocity_estimator.py` via `server_x3.py`'s pose callback; use the drift-free scan-matched pose to transform depth centroids to global coordinates, eliminating mecanum slip distortion from track history windows.
*   **76. Idea 191: Speed-Adaptive Forward LiDAR Detection Range** (Medium-High ROI) — Scale forward blockage threshold in `_update_bypass_offset` as `look_ahead = max(0.75, last_vx_cmd / KP_DIST + 0.30)` to provide proportionally earlier obstacle detection at higher commanded speeds.
*   **77. Idea 149: Yaw-Aware Dynamic Repulsion & Clearance-Based Rotation Center Shifting** (Medium ROI) — Project chassis corner vertices into the LiDAR frame during `ROTATE_180`/`ROTATE_HOME`; inject corrective lateral strafe velocity if any corner violates the 0.30 m wall clearance bound.
*   **78. Idea 193: Asymmetric Lateral Acceleration/Deceleration Rate Limiting** (Medium ROI) — Apply higher deceleration cap (1.5 m/s²) when `vy_cmd` opposes the APF repulsion direction vs. the current 0.5 m/s² symmetric limit in `DRIVE_TO_B`/`DRIVE_TO_A`.
*   **79. Idea 153: LiDAR Range-Cluster Guided Depth ROI Extraction** (Medium ROI) — Project 2D LiDAR obstacle clusters into the camera FOV and run `_extract_depth_centroids` only within those cropped bounding boxes, reducing full-frame depth scanning overhead.
*   **80. Idea 156: Real-Time Control Jitter and Energy Efficiency Metrics Logger** (Medium ROI) — Append control loop step-interval variance, cumulative squared cross-track error, and integrated velocity magnitude to the CSV logger in `ab_comparison_test.py` at existing `LOG_HZ`.
*   **81. Idea 154: Predictive Trajectory-Intersector Bypassing** (Medium ROI) — Compute the intersection of the robot's path and the pedestrian's predicted velocity vector; set `target_lateral_offset` to steer the robot behind the pedestrian's projected crossing path rather than cutting in front.
*   **82. Idea 195: MLP Prediction Variance-Based Confidence Weighting** (Medium ROI) — Maintain an EMA variance estimate of `speed` per track; weight TTC contribution as `effective_speed = speed * exp(-k * var)` in `_get_speed_scaling()` to dampen noisy tracks without disabling them.
*   **83. Idea 196: Dynamic Track Association Radius Based on Last Estimated Speed** (Medium ROI) — Replace fixed `max_dist = 0.8m` in `ObstacleTracker.update()` with `match_radius = max(0.3, min(0.8, last_speed * dt * 2.0 + 0.15))` to reduce identity-switch errors for slow objects while maintaining tracking for fast pedestrians.
*   **84. Idea 164: Corridor Intersection Detection via Angular LiDAR Entropy** (Medium ROI) — Monitor Shannon entropy of range returns in side 90° sectors; a sudden increase indicates a corridor intersection; automatically scale down forward speed before emerging obstacles appear.
*   **85. Idea 159: RANSAC Line-Fitting for Corridor Wall Boundary Identification** (Medium ROI) — Fit continuous wall lines in left/right LiDAR sectors using a simple RANSAC line fit; compute repulsion forces relative to the fitted wall rather than the raw single-point minimum clearance to stabilize APF centering.
*   **86. Idea 174: Predictive Deceleration Profiling for Dynamic Obstacle Crossings** (Medium ROI) — Estimate the pedestrian's path crossing time and robot arrival time; proactively reduce commanded speed before the crossing to avoid abrupt emergency stops.
*   **87. Idea 140: Multi-Track Kalman Filter Prediction for MLP Window Alignment** (Medium ROI) — Back each `ObstacleTracker` track with a simple 2D constant-velocity Kalman Filter; use the Kalman prediction to dead-reckon a missed frame's position and insert it into the history queue, guaranteeing uniform 10 Hz spacing for the MLP.
*   **88. Idea 161: Bounding Box Temporal Prediction Gating for Slow YOLO Framerates** (Medium ROI) — Project the last YOLO person bounding box forward using estimated velocity when new YOLO frames are not yet available, enabling the depth-centroid gating in `_extract_depth_centroids` to run at 10 Hz even when YOLO is throttled to 2 Hz.
*   **89. Idea 163: Bilateral Depth Filtering for Doorway Contour Preservation** (Medium ROI) — Apply a fast 1D bilateral filter on raw depth rows before the voxel downsampling step in `_extract_depth_centroids` to smooth noise while preserving sharp depth edges at doorway boundaries.
*   **90. Idea 176: Non-Blocking Asynchronous Telemetry Logger** (Medium ROI) — Push `RunLogger.rows` entries to a `queue.Queue`; have a background daemon thread drain the queue to disk, preventing CSV flush operations from blocking the main 20 Hz control loop.
*   **91. Idea 179: Proactive Deceleration Profiling for Lateral Wall Clearances** (Medium ROI) — Scale `MAX_LINEAR_SPEED` proportionally to current wall clearance as `max_speed = BASE * min(1.0, wall_clearance / safety_threshold)` in tight sections, giving the lateral controller more time to correct slip.
*   **92. Idea 186: Temporal Tracking Gate Hysteresis via Multi-Frame Bounding Box Matching** (Medium ROI) — Continue gating a depth centroid track against the last known YOLO bounding box (expanded by a search margin) for up to 3 occlusion frames before dropping it, preventing track loss during brief visual frame drops.
*   **93. Idea 184: Corridor Cornering Clearance Compensation via Sweep Envelope Expansion** (Medium ROI) — Expand safety clearance bounds proportionally to `abs(omega_z)` during turnaround states; inject counter-lateral mecanum velocities if the dynamic sweep footprint intersects wall boundaries.
*   **94. Idea 181: Double-Buffered Shared Memory IPC** (Medium ROI) — Allocate two shared memory blocks for BGR and depth frames; maintain a control byte indicating the latest completed buffer index to provide lock-free, race-free IPC between the ROS2 camera callback and the WebSocket broadcast loop.
*   **95. Idea 173: Self-Adapting Contrast Enhancement (CLAHE) for Shadow Exclusions** (Medium-Low ROI) — Apply local CLAHE on the depth confidence or IR channel before thresholding to preserve obstacle outlines in dark corridor corners, preventing tracking dropout in shadowed regions.
*   **96. Idea 169: Dynamic Corridor Angle Yaw-Alignment Control** (Medium-Low ROI) — Extract corridor wall orientation from fitted LiDAR RANSAC lines and dynamically adjust `self.target_yaw` to stay parallel to the local hallway, reducing steering oscillations in slightly off-axis starts.
*   **97. Idea 165: Multi-Threaded Frame-Fetch Gating (Producer-Consumer Queue)** (Medium-Low ROI) — Run `get_depth_frame()` and `get_raw_depth_frame()` in a separate producer thread writing to a single-element buffer; the inference loop fetches non-blockingly, preventing DDS latency from stalling control updates.
*   **98. Idea 185: Vectorized LiDAR Scan Compaction via Adaptive Decimation** (Medium-Low ROI) — Decimate the 360-beam scan down to ~90 rays in flat sectors (range gradient < threshold) while maintaining full resolution near obstacles; reduces ICP and APF input point counts.
*   **99. Idea 180: Vectorized LiDAR Corner/Vertex Extraction via RDP Line Simplification** (Medium-Low ROI) — Apply the Ramer-Douglas-Peucker algorithm to the LiDAR coordinate array to extract 4–6 wall/corner vertices; execute APF repulsion and blockage checks on the simplified vertices only.
*   **100. Idea 183: K-Means Guided Active Depth Cropping ROI for Crowded Scenarios** (Medium-Low ROI) — Apply 1D K-means clustering on raw depth rows to identify distinct depth layers; create separate depth mask ROIs per cluster to keep close-proximity targets isolated during contour processing.
*   **101. Idea 168: Reflectivity Filtering for Specular Ground and Metallic Surface Exclusions** (Medium-Low ROI) — Filter LiDAR range points whose return intensity falls below the diffuse surface threshold (specular floors/glass exhibit atypical profiles), excluding ghost obstacle readings.
*   **102. Idea 182: Track-Frame Heading Normalization for MLP Generalization** (Medium-Low ROI) — Apply PCA to each track's history coordinates to determine its primary motion axis; rotate the window to align that axis with the local x-axis before feature extraction in `_build_window_features`.
*   **103. Idea 206: Static Fixture Tagging via EMA Motion Score** (Medium-High ROI) — Maintain a per-track `motion_score = 0.8 * prev + 0.2 * (|dx| + |dy|)` in `ObstacleTracker.update()`; tag any track with `motion_score < 0.004 m/frame` sustained for 20+ frames as `is_static_fixture` and skip it in `_get_speed_scaling` and bypass checks, eliminating inference overhead from static furniture tracks without modifying the depth extraction pipeline.
*   **104. Idea 210: Depth Colorization Lazy Gating by Client Subscription State** (Medium-High ROI) — Split `ROS2Bridge._depth_cb` to always store the raw float32 array (required by `VelocityEstimator`) but only compute the colorized BGR frame when `depth_enabled = True`, eliminating the 3–5 ms colorization cost per frame during normal operation when depth visualization is inactive.
*   **105. Idea 217: Scale `vy_rep` Proportionally to `max_allowed_offset` in Tight Corridors** (Medium ROI) — Clamp `vy_rep` to `[-max_allowed_offset * KP_LATERAL, max_allowed_offset * KP_LATERAL]` in `_update_bypass_offset` so APF repulsion stays proportionally bounded to available corridor width, preventing the fixed 0.12 m/s cap from driving the robot past computed corridor bounds in tight spaces.
*   **106. Idea 214: APF Repulsion Saturation to Prevent Anti-Bypass Interference** (Medium ROI) — In `_update_bypass_offset`, zero `vy_rep` when its sign opposes the active `target_lateral_offset` and its magnitude is less than the bypass offset command, preventing wall repulsion from fighting active bypass maneuvers while still protecting against wall overshoots after the bypass target is reached.
*   **107. Idea 215: Bypass Clearance Confirmation Countdown to Suppress Re-Engagement Oscillation** (Medium ROI) — Add a `_clear_confirm_count` counter requiring the path to be consistently confirmed clear for 3–5 consecutive ticks (150–250 ms) before resetting `target_lateral_offset = 0.0`, eliminating rapid bypass/center toggling for pedestrians near the edge of the detection zone.
*   **108. Idea 216: Dual Depth Frame Acquisition via Single `ROS2Bridge` Lock** (Medium ROI) — Add a `get_depth_frames()` method to `ROS2Bridge` returning `(coloured_depth, raw_depth)` in a single `with self._lock:` block; replace the two sequential `get_depth_frame()` / `get_raw_depth_frame()` calls in `_inference_loop` with this combined fetch, halving lock overhead for depth retrieval at 10 Hz.
*   **109. Idea 218: Z-Score Outlier Rejection for Centroid Depth Median Accuracy** (Medium ROI) — In `_extract_depth_centroids`, after computing an initial `np.median(valid_depths)`, remove samples where `|depth - median| > 1.5 * std(valid_depths)` before recomputing the final median, reducing wall-pixel contamination bias in the 3D centroid Z coordinate fed to the MLP history window.
*   **110. Idea 219: ICP Warm-Start from Previous Residual Correction for Lateral Slip Compensation** (Medium ROI) — Store the last accepted ICP residual and blend it as a warm-start prior via `initial_dx += alpha * prev_dx_residual` (α ≈ 0.3) to bias alignment toward the persistent mecanum lateral slip direction, reducing convergence iterations and improving correction accuracy during sustained bypass drives.
*   **111. Idea 220: Integral Cross-Track Error Term for Steady-State Lateral Drift Elimination** (Medium ROI) — Add an integral term `vy_i += KI_LATERAL * (target_lateral_offset - path_y) * dt` (KI ≈ 0.1) with anti-windup clamp at ±0.05 m/s to the lateral P controller in drive states, driving steady-state cross-track error to zero and keeping the robot precisely centered on the configured offset across repeated drive segments.

---

## 4. Low-ROI Tier (Rank 112 - 131)
*These ideas involve complex mathematics, multi-sensor clustering fusion, retraining models with variable dimensions, or significant architectural rewrites, but yield minor returns for this front-corridor A/B demo.*

*   **75. Idea 4: Hybrid Kalman Filter Tracking & MLP Predictor Fusion**
*   **76. Idea 5: Spatio-Temporal Trajectory Rollouts for Costmap Updates**
*   **77. Idea 23: Sensor Fusion using 2D LiDAR Point Cloud Clustering**
*   **78. Idea 49: LiDAR-Camera Joint Frustum Association and FOV Handover**
*   **79. Idea 10: Sensor Failure Fallback and Dynamic Source Re-routing**
*   **80. Idea 12: Non-Linear Pedestrian Path Modeling via Spline Extrapolations**
*   **81. Idea 27: Multi-Pedestrian Priority-Based Local Waypoint Halting**
*   **82. Idea 39: Map-Masked LiDAR Front Blockage Filtering**
*   **83. Idea 60: Global SLAM-Path Waypoint Interpolation Recovery**
*   **84. Idea 59: Focal-Length Bounding Box Projection Fallback for Blind Spots**
*   **85. Idea 69: Multi-Sensor Boundary Contrast Cross-Checking**
*   **86. Idea 74: Boundary-Truncation Reconstruction and Shape Fitting**
*   **87. Idea 75: Velocity Obstacle (VO) Vector Selection for Holonomic Navigation**
*   **88. Idea 76: Decoupled Background YOLO Throttling with Velocity Projection**
*   **89. Idea 6: Auto-Tuning/Self-Calibration of Controller Gains**
*   **90. Idea 8: Dynamic Window Sizing for High-Acceleration Pedestrians**
*   **91. Idea 37: Native Raw Depth Retrieval for AstraCamera**
*   **92. Idea 41: Vectorized Feature Construction in Track History Processing**
*   **93. Idea 47: Event-Driven or Throttled Bypass Optimization**
*   **94. Idea 55: Predictive Vector-Field Orientation Alignment**
*   **95. Idea 61: Map Hash Checksumming and Delta Compression**
*   **96. Idea 62: Pre-Allocated Static Tracks and Numpy Deque Buffers**
*   **97. Idea 64: Depth-Gradient Edge Segmentation for Overlapping Obstacles**
*   **98. Idea 73: Multi-Scale Temporal Resolution Input Feature Vector**
*   **99. Idea 68: Kinematic Back-Propagation for Track Initialization Padding**
*   **100. Idea 50: Footprint-Swept Corridor Expansion for Holonomic Path Protection**
*   **101. Idea 25: Dynamic Costmap Overlay on Map Canvas in GUI**
*   **102. Idea 89: Ground Plane RANSAC Filtering for Leg Contour Isolation**
*   **103. Idea 94: Spatio-Temporal Static Saliency Masking for Doorways**
*   **104. Idea 98: Barycentric Relative Coordinate Stabilization**
*   **105. Idea 100: Spatio-Temporal Gap Selection (Window of Opportunity)**
*   **106. Idea 103: Model-Prediction Autoregressive Feedback Loop**
*   **107. Idea 113: Edge-Preserving Bilateral Filtering for Centroid Stability**
*   **108. Idea 114: Vertical Wall RANSAC Filtering**
*   **109. Idea 132: PyTorch JIT Optimization via TensorRT Compilation (Torch-TensorRT)**
*   **110. Idea 139: Dynamic Camera Gain & Auto-Exposure Control for Shadow Adaptability** — Requires OpenNI2 or V4L2 exposure commands not exposed by the current `AstraCamera` driver; benefit is mild given raw depth thresholding is already lighting-invariant.
*   **111. Idea 142: Gated Recurrent Unit (GRU) Temporal Feature Encoding** — Replaces the entire MLP architecture; requires complete model re-collection, retraining, and TorchScript export pipeline; significant effort for uncertain gain over the fine-tuned MLP.
*   **112. Idea 148: Depth Gradient Watershed Segmentation for Close-Proximity Obstacle Separation** — Marker-controlled watershed is computationally expensive on float32 depth frames; the close-proximity pedestrian-wall merging case is already partially handled by contour cropping and voxel gating.
*   **113. Idea 151: Auto-Calibrating Homography & Projection Alignment via YOLO Centroid Feedback** — Recursive least squares projection calibration adds complex state tracking; camera mount tolerances are stable on a rigid robot chassis and static recalibration is sufficient.
*   **114. Idea 157: Adaptively Adjusted Online Feature Normalization Gating** — Online mean/variance tracking adds per-frame arithmetic and requires careful initialization logic; the fine-tuned scaler already handles the deployment domain well.
*   **115. Idea 158: LiDAR-Depth Dynamic Extrinsics Auto-Tuning via Ground-Plane Invariance** — Real-time tilt correction of camera-LiDAR extrinsics requires continuous IMU integration and is sensitive to calibration quality; the current fusion gate uses 15 px padding margins to absorb minor misalignments.
*   **116. Idea 162: Ego-Velocity-Weighted Feature Regularization** — Expanding the MLP input from 40 to 43 features requires full retraining; the EKF ego-motion subtraction already accounts for most base kinematic distortion.
*   **117. Idea 167: Relative Acceleration Input Features for Intercept Stability** — Requires model retraining with expanded feature vector; acceleration features derived from noisy depth centroid positions amplify rather than reduce input noise without careful smoothing.
*   **118. Idea 170: Direct Tensor Sharing via CUDA IPC (Unified GPU Memory Pipeline)** — The MLP is tiny (40→256→128→64→2) and runs in < 0.5 ms on CPU; CUDA IPC setup complexity far outweighs any latency gain for this model size.
*   **119. Idea 172: Multi-Modal Model Fusion (LiDAR-Feature Combined MLP)** — Requires complete redesign of the model input pipeline, new LiDAR feature extraction at inference time, and full retraining; LiDAR shape features for pedestrians are unreliable at 8 Hz with 2D scan geometry.
*   **120. Idea 177: Multi-Scale Temporal Feature Pooling (Pyramid Temporal History Window)** — Requires model retraining with expanded input dimensions; the existing WINDOW_SIZE=10 covers sufficient history for corridor-speed pedestrians; marginal accuracy gain vs. retraining cost.
*   **121. Idea 178: LiDAR Intensity-based Dynamic Floor Segment Filter** — YDLidar X3 returns do not include per-point reflectivity intensity in the ROS2 `LaserScan` message; feature is hardware-dependent and not available with the current driver.
*   **122. Idea 192: Twist Message Object Pre-Allocation in Control Loop** — ROS2 C++ Twist message bindings are extremely fast to construct in Python; the per-iteration allocation overhead is negligible compared to ROS2 publish overhead; minimal return for any investment.
