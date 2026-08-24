---
title: Idea Index
tags: [moc, idea]
---

# Idea Index

**Generated — do not edit.** Run `python3 scripts/split_ideas.py` to refresh, 
`--check` to verify. One note per idea lives in `ideas/notes/`; the monolithic 
logs remain the source of truth.

546 ideas across 4 logs — **54 with implementation evidence**, 492 still candidates.

> [!warning] Legacy ids collide
> `June_architectural_ideas.md` numbers ideas 1–340 and `June_performance_ideas.md`
> numbers a *different* set 1–120. A bare "Idea 81" is ambiguous. Use the namespaced
> ids below (`JA-081` vs `JP-081`); they are unique across the whole corpus.

> [!note] How status is decided
> No idea log marks anything as done. `Implemented` here means the idea is listed in
> `June_roi_analysis.md` → *Already Implemented Ideas*, or its number appears in a
> source comment. Both apply only to the `JA` namespace, which is the numbering those
> two records use — so a `JP`/`J`/`A` idea showing `Logged` means *no evidence was
> looked for*, not that it was rejected.

## June — Architectural (`JA-`)

340 ideas, 54 implemented — source: `ideas/June_architectural_ideas.md`

| ID | Title | Status | ROI | Domain |
| :-- | :-- | :-- | :-- | :-- |
| [[JA-001-ego-motion-compensation-in-velocityestimator\|JA-001]] | Ego-Motion Compensation in `VelocityEstimator` | ✅ Implemented | — | — |
| [[JA-002-live-ros2-topic-publishing-for-nav2-mppi-integra\|JA-002]] | Live ROS2 Topic Publishing for Nav2/MPPI Integration | ✅ Implemented | — | — |
| [[JA-003-pd-based-closed-loop-heading-control-in-calibrat\|JA-003]] | PD-based Closed-Loop Heading Control in Calibration Nodes | ✅ Implemented | — | — |
| [[JA-004-hybrid-kalman-filter-tracking-mlp-predictor-fusi\|JA-004]] | Hybrid Kalman Filter Tracking & MLP Predictor Fusion | candidate | — | — |
| [[JA-005-spatio-temporal-trajectory-rollouts-for-costmap\|JA-005]] | Spatio-Temporal Trajectory Rollouts for Costmap Updates | candidate | — | — |
| [[JA-006-auto-tuning-self-calibration-of-controller-gains\|JA-006]] | Auto-Tuning/Self-Calibration of Controller Gains | candidate | — | — |
| [[JA-007-adaptive-lidar-outlier-filtering-in-indoor-class\|JA-007]] | Adaptive LiDAR Outlier Filtering in Indoor Classrooms | candidate | — | — |
| [[JA-008-dynamic-window-sizing-for-high-acceleration-pede\|JA-008]] | Dynamic Window Sizing for High-Acceleration Pedestrians | candidate | — | — |
| [[JA-009-rviz2-spatio-temporal-trajectory-velocity-vector\|JA-009]] | RViz2 Spatio-Temporal Trajectory & Velocity Vector Markers | ✅ Implemented | — | — |
| [[JA-010-sensor-failure-fallback-and-dynamic-source-re-ro\|JA-010]] | Sensor Failure Fallback and Dynamic Source Re-routing | candidate | — | — |
| [[JA-011-encoders-imu-slip-compensation-using-ekf-velocit\|JA-011]] | Encoders-IMU Slip Compensation using EKF Velocity Feedback | ✅ Implemented | — | — |
| [[JA-012-non-linear-pedestrian-path-modeling-via-spline-e\|JA-012]] | Non-Linear Pedestrian Path Modeling via Spline Extrapolations | candidate | — | — |
| [[JA-013-dynamic-speed-scaling-based-on-pedestrian-proxim\|JA-013]] | Dynamic Speed Scaling based on Pedestrian Proximity and Time-to-Collision (TTC) | ✅ Implemented | — | — |
| [[JA-014-temporal-jitter-mitigation-in-a-b-run-logs-via-s\|JA-014]] | Temporal Jitter Mitigation in A/B Run Logs via Synced Message Buffering | candidate | — | — |
| [[JA-015-path-yielding-and-recovery-states-for-blocked-tr\|JA-015]] | Path Yielding and Recovery States for Blocked Trajectories | candidate | — | — |
| [[JA-016-bi-directional-telemetry-and-progress-reporting\|JA-016]] | Bi-Directional Telemetry and Progress Reporting for A/B Runs | candidate | — | — |
| [[JA-017-subprocess-lifecycle-management-and-orphan-preve\|JA-017]] | Subprocess Lifecycle Management and Orphan Prevention | ✅ Implemented | — | — |
| [[JA-018-cross-track-error-correction-using-holonomic-str\|JA-018]] | Cross-Track Error Correction Using Holonomic Strafing | ✅ Implemented | — | — |
| [[JA-019-depth-aware-3d-centroid-reconstruction-for-veloc\|JA-019]] | Depth-Aware 3D Centroid Reconstruction for Velocity Scaling | ✅ Implemented | — | — |
| [[JA-020-temporal-consistency-and-interpolation-in-track\|JA-020]] | Temporal Consistency and Interpolation in Track History | candidate | — | — |
| [[JA-021-direct-meters-based-depth-processing-to-avoid-no\|JA-021]] | Direct Meters-Based Depth Processing to Avoid Normalization Artifacts | ✅ Implemented | — | — |
| [[JA-022-pedestrian-clearance-distance-logging-for-quanti\|JA-022]] | Pedestrian Clearance Distance Logging for Quantitative Navigation Metrics | ✅ Implemented | — | — |
| [[JA-023-sensor-fusion-using-2d-lidar-point-cloud-cluster\|JA-023]] | Sensor Fusion using 2D LiDAR Point Cloud Clustering | candidate | — | — |
| [[JA-024-slam-based-odometry-drift-logging-for-ground-tru\|JA-024]] | SLAM-Based Odometry Drift Logging for Ground-Truth Evaluation | candidate | — | — |
| [[JA-025-dynamic-costmap-overlay-on-map-canvas-in-gui\|JA-025]] | Dynamic Costmap Overlay on Map Canvas in GUI | candidate | — | — |
| [[JA-026-heading-drift-correction-via-slam-loop-closures\|JA-026]] | Heading Drift Correction via SLAM Loop Closures in Test Run Recovery | ✅ Implemented | — | — |
| [[JA-027-multi-pedestrian-priority-based-local-waypoint-h\|JA-027]] | Multi-Pedestrian Priority-Based Local Waypoint Halting | candidate | — | — |
| [[JA-028-dynamic-look-ahead-distance-for-heading-correcti\|JA-028]] | Dynamic Look-Ahead Distance for Heading Corrections | candidate | — | — |
| [[JA-029-adaptive-lateral-bypass-direction-selection\|JA-029]] | Adaptive Lateral Bypass Direction Selection | ✅ Implemented | — | — |
| [[JA-030-smoothed-lateral-velocity-profiles-for-mecanum-s\|JA-030]] | Smoothed Lateral Velocity Profiles for Mecanum Slip Prevention | ✅ Implemented | — | — |
| [[JA-031-dynamic-path-re-planning-timeout-scaling\|JA-031]] | Dynamic Path Re-planning Timeout Scaling | ✅ Implemented | — | — |
| [[JA-032-damping-gain-scheduling-for-lateral-alignment\|JA-032]] | Damping Gain Scheduling for Lateral Alignment | candidate | — | — |
| [[JA-033-dynamic-temporal-scaling-of-centroid-displacemen\|JA-033]] | Dynamic Temporal Scaling of Centroid Displacements | candidate | — | — |
| [[JA-034-distance-adaptive-area-thresholding-for-near-fie\|JA-034]] | Distance-Adaptive Area Thresholding for Near-Field Detections | candidate | — | — |
| [[JA-035-continuous-gap-based-lateral-bypass-offset-calcu\|JA-035]] | Continuous, Gap-Based Lateral Bypass Offset Calculation | candidate | — | — |
| [[JA-036-cropped-contour-masking-for-centroid-extraction\|JA-036]] | Cropped Contour Masking for Centroid Extraction | ✅ Implemented | — | — |
| [[JA-037-native-raw-depth-retrieval-for-astracamera\|JA-037]] | Native Raw Depth Retrieval for AstraCamera | candidate | — | — |
| [[JA-038-velocity-projected-constant-velocity-tracker-ass\|JA-038]] | Velocity-Projected Constant Velocity Tracker Association | candidate | — | — |
| [[JA-039-map-masked-lidar-front-blockage-filtering\|JA-039]] | Map-Masked LiDAR Front Blockage Filtering | candidate | — | — |
| [[JA-040-swept-volume-rotational-safety-fields\|JA-040]] | Swept-Volume Rotational Safety Fields | candidate | — | — |
| [[JA-041-vectorized-feature-construction-in-track-history\|JA-041]] | Vectorized Feature Construction in Track History Processing | candidate | — | — |
| [[JA-042-shell-sourcing-elimination-for-ros2-subprocesses\|JA-042]] | Shell Sourcing Elimination for ROS2 Subprocesses | candidate | — | — |
| [[JA-043-temporal-depth-filtering-for-centroid-smoothing\|JA-043]] | Temporal Depth Filtering for Centroid Smoothing | ✅ Implemented | — | — |
| [[JA-044-vertical-centroid-fusion-for-partially-occluded\|JA-044]] | Vertical Centroid Fusion for Partially Occluded Pedestrians | candidate | — | — |
| [[JA-045-blended-diagonal-bypass-path-profiling\|JA-045]] | Blended Diagonal Bypass Path Profiling | ✅ Implemented | — | — |
| [[JA-046-batched-mlp-inference-for-multi-track-scaling\|JA-046]] | Batched MLP Inference for Multi-Track Scaling | ✅ Implemented | — | — |
| [[JA-047-event-driven-or-throttled-bypass-optimization\|JA-047]] | Event-Driven or Throttled Bypass Optimization | candidate | — | — |
| [[JA-048-local-trajectory-translation-normalization\|JA-048]] | Local Trajectory Translation Normalization | ✅ Implemented | — | — |
| [[JA-049-lidar-camera-joint-frustum-association-and-fov-h\|JA-049]] | LiDAR-Camera Joint Frustum Association and FOV Handover | candidate | — | — |
| [[JA-050-footprint-swept-corridor-expansion-for-holonomic\|JA-050]] | Footprint-Swept Corridor Expansion for Holonomic Path Protection | candidate | — | — |
| [[JA-051-client-side-drawing-overlay-delegation\|JA-051]] | Client-Side Drawing Overlay Delegation | candidate | — | — |
| [[JA-052-decimated-depth-arrays-for-centroid-median-calcu\|JA-052]] | Decimated Depth Arrays for Centroid Median Calculation | ✅ Implemented | — | — |
| [[JA-053-bipartite-matching-for-tracker-centroid-associat\|JA-053]] | Bipartite Matching for Tracker Centroid Association | candidate | — | — |
| [[JA-054-morphological-flood-fill-hole-correction-for-spe\|JA-054]] | Morphological Flood-Fill Hole Correction for Specular Dropouts | candidate | — | — |
| [[JA-055-predictive-vector-field-orientation-alignment\|JA-055]] | Predictive Vector-Field Orientation Alignment | candidate | — | — |
| [[JA-056-simd-optimized-opencv-inrange-thresholding\|JA-056]] | SIMD-Optimized OpenCV inRange Thresholding | ✅ Implemented | — | — |
| [[JA-057-binary-serialization-of-depth-frames-via-websock\|JA-057]] | Binary Serialization of Depth Frames via WebSockets | candidate | — | — |
| [[JA-058-kinematic-derivative-enrichment-for-mlp-input-ve\|JA-058]] | Kinematic Derivative Enrichment for MLP Input Vectors | candidate | — | — |
| [[JA-059-focal-length-bounding-box-projection-fallback-fo\|JA-059]] | Focal-Length Bounding Box Projection Fallback for Blind Spots | candidate | — | — |
| [[JA-060-global-slam-path-waypoint-interpolation-recovery\|JA-060]] | Global SLAM-Path Waypoint Interpolation Recovery | candidate | — | — |
| [[JA-061-map-hash-checksumming-and-delta-compression\|JA-061]] | Map Hash Checksumming and Delta Compression | candidate | — | — |
| [[JA-062-pre-allocated-static-tracks-and-numpy-deque-buff\|JA-062]] | Pre-Allocated Static Tracks and Numpy Deque Buffers | candidate | — | — |
| [[JA-063-robust-coordinate-clamping-and-velocity-bound-co\|JA-063]] | Robust Coordinate Clamping and Velocity Bound Constraints | ✅ Implemented | — | — |
| [[JA-064-depth-gradient-edge-segmentation-for-overlapping\|JA-064]] | Depth-Gradient Edge Segmentation for Overlapping Obstacles | candidate | — | — |
| [[JA-065-forward-projected-path-collision-corridors\|JA-065]] | Forward-Projected Path Collision Corridors | candidate | — | — |
| [[JA-066-onnx-runtime-conversion-for-pytorch-free-executi\|JA-066]] | ONNX Runtime Conversion for PyTorch-Free Execution | candidate | — | — |
| [[JA-067-battery-voltage-query-throttling-in-ros2-bridge\|JA-067]] | Battery Voltage Query Throttling in ROS2 Bridge | ✅ Implemented | — | — |
| [[JA-068-kinematic-back-propagation-for-track-initializat\|JA-068]] | Kinematic Back-Propagation for Track Initialization Padding | candidate | — | — |
| [[JA-069-multi-sensor-boundary-contrast-cross-checking\|JA-069]] | Multi-Sensor Boundary Contrast Cross-Checking | candidate | — | — |
| [[JA-070-omnidirectional-direction-of-travel-coordinate-p\|JA-070]] | Omnidirectional Direction-of-Travel Coordinate Projection | ✅ Implemented | — | — |
| [[JA-071-nav2-action-status-query-throttling\|JA-071]] | Nav2 Action Status Query Throttling | ✅ Implemented | — | — |
| [[JA-072-pure-numpy-feature-normalization-eliminating-sci\|JA-072]] | Pure NumPy Feature Normalization (Eliminating Scikit-Learn) | ✅ Implemented | — | — |
| [[JA-073-multi-scale-temporal-resolution-input-feature-ve\|JA-073]] | Multi-Scale Temporal Resolution Input Feature Vector | candidate | — | — |
| [[JA-074-boundary-truncation-reconstruction-and-shape-fit\|JA-074]] | Boundary-Truncation Reconstruction and Shape Fitting | candidate | — | — |
| [[JA-075-velocity-obstacle-vo-vector-selection-for-holono\|JA-075]] | Velocity Obstacle (VO) Vector Selection for Holonomic Navigation | candidate | — | — |
| [[JA-076-decoupled-background-yolo-throttling-with-veloci\|JA-076]] | Decoupled Background YOLO Throttling with Velocity Projection | candidate | — | — |
| [[JA-077-connected-components-with-stats-for-centroid-ext\|JA-077]] | Connected Components with Stats for Centroid Extraction | candidate | — | — |
| [[JA-078-kinematic-acceleration-limiting-output-filter\|JA-078]] | Kinematic Acceleration-Limiting Output Filter | candidate | — | — |
| [[JA-079-semantic-gating-with-yolo-bounding-boxes\|JA-079]] | Semantic Gating with YOLO Bounding Boxes | candidate | — | — |
| [[JA-080-rear-sector-lidar-protection-for-backing-recover\|JA-080]] | Rear-Sector LiDAR Protection for Backing Recovery | ✅ Implemented | — | — |
| [[JA-081-immediate-depth-downsampling-for-centroid-extrac\|JA-081]] | Immediate Depth Downsampling for Centroid Extraction | ✅ Implemented | — | — |
| [[JA-082-dynamic-subscription-lifecycle-management-for-id\|JA-082]] | Dynamic Subscription Lifecycle Management for Idle Power Savings | candidate | — | — |
| [[JA-083-hardware-triggered-pose-timestamp-interpolation\|JA-083]] | Hardware-Triggered Pose-Timestamp Interpolation | candidate | — | — |
| [[JA-084-edge-proximity-confidence-weighting-for-fov-tran\|JA-084]] | Edge-Proximity Confidence Weighting for FOV Transitions | candidate | — | — |
| [[JA-085-predictive-dynamic-corridor-shifting\|JA-085]] | Predictive Dynamic Corridor Shifting | candidate | — | — |
| [[JA-086-adaptive-duty-cycle-throttling-in-estimator-idle\|JA-086]] | Adaptive Duty-Cycle Throttling in Estimator Idle States | candidate | — | — |
| [[JA-087-high-performance-orjson-serialization\|JA-087]] | High-Performance orjson Serialization | ✅ Implemented | — | — |
| [[JA-088-heading-aligned-path-rotation-for-trajectory-inv\|JA-088]] | Heading-Aligned Path Rotation for Trajectory Invariance | candidate | — | — |
| [[JA-089-ground-plane-ransac-filtering-for-leg-contour-is\|JA-089]] | Ground Plane RANSAC Filtering for Leg Contour Isolation | candidate | — | — |
| [[JA-090-hysteresis-based-speed-scaling-for-deceleration\|JA-090]] | Hysteresis-Based Speed Scaling for Deceleration Smoothing | candidate | — | — |
| [[JA-091-vectorized-pairwise-distance-matrix-broadcasting\|JA-091]] | Vectorized Pairwise Distance Matrix Broadcasting | ✅ Implemented | — | — |
| [[JA-092-pre-allocated-lut-map-pixel-mapping\|JA-092]] | Pre-Allocated LUT Map Pixel Mapping | candidate | — | — |
| [[JA-093-mlp-velocity-driven-dead-reckoning-for-occluded\|JA-093]] | MLP-Velocity-Driven Dead Reckoning for Occluded Tracks | candidate | — | — |
| [[JA-094-spatio-temporal-static-saliency-masking-for-door\|JA-094]] | Spatio-Temporal Static Saliency Masking for Doorways | ✅ Implemented | — | — |
| [[JA-095-dynamic-side-clearance-potential-fields\|JA-095]] | Dynamic Side-Clearance Potential Fields | candidate | — | — |
| [[JA-096-precomputed-pinhole-projection-grid-projection-l\|JA-096]] | Precomputed Pinhole Projection Grid (Projection LUT) | candidate | — | — |
| [[JA-097-decoupled-high-low-frequency-telemetry-split\|JA-097]] | Decoupled High/Low Frequency Telemetry Split | candidate | — | — |
| [[JA-098-barycentric-relative-coordinate-stabilization\|JA-098]] | Barycentric Relative Coordinate Stabilization | candidate | — | — |
| [[JA-099-depth-histogram-peak-slicing-for-dynamic-segment\|JA-099]] | Depth Histogram Peak-Slicing for Dynamic Segmenting | candidate | — | — |
| [[JA-100-spatio-temporal-gap-selection-window-of-opportun\|JA-100]] | Spatio-Temporal Gap Selection (Window of Opportunity) | candidate | — | — |
| [[JA-101-pre-allocated-numpy-ring-buffers-for-coordinate\|JA-101]] | Pre-Allocated NumPy Ring Buffers for Coordinate History | candidate | — | — |
| [[JA-102-event-driven-camera-synced-broadcast-loop\|JA-102]] | Event-Driven Camera-Synced Broadcast Loop | candidate | — | — |
| [[JA-103-model-prediction-autoregressive-feedback-loop\|JA-103]] | Model-Prediction Autoregressive Feedback Loop | candidate | — | — |
| [[JA-104-temporal-inter-frame-depth-differencing-motion-m\|JA-104]] | Temporal Inter-Frame Depth Differencing (Motion Masking) | candidate | — | — |
| [[JA-105-active-center-point-drift-correction-during-rota\|JA-105]] | Active Center-Point Drift Correction during Rotation | candidate | — | — |
| [[JA-106-downscaled-depth-visualization-encoding\|JA-106]] | Downscaled Depth Visualization Encoding | ✅ Implemented | — | — |
| [[JA-107-double-exponential-smoothing-holt-s-linear-trend\|JA-107]] | Double Exponential Smoothing (Holt's Linear Trend) | candidate | — | — |
| [[JA-108-kinematic-stop-trigger-gating\|JA-108]] | Kinematic Stop-Trigger Gating | ✅ Implemented | — | — |
| [[JA-109-centroid-presence-verification-filter-track-init\|JA-109]] | Centroid Presence Verification Filter (Track Initiation Gate) | ✅ Implemented | — | — |
| [[JA-110-dynamic-corridor-width-clearance-scaling\|JA-110]] | Dynamic Corridor Width Clearance Scaling | ✅ Implemented | — | — |
| [[JA-111-serial-command-write-rate-optimization-30hz-thro\|JA-111]] | Serial Command Write Rate Optimization (30Hz Throttle) | ✅ Implemented | — | — |
| [[JA-112-single-precision-float32-scale-arithmetic\|JA-112]] | Single-Precision float32 Scale Arithmetic | candidate | — | — |
| [[JA-113-edge-preserving-bilateral-filtering-for-centroid\|JA-113]] | Edge-Preserving Bilateral Filtering for Centroid Stability | candidate | — | — |
| [[JA-114-vertical-wall-ransac-filtering\|JA-114]] | Vertical Wall RANSAC Filtering | candidate | — | — |
| [[JA-115-live-lidar-wall-relative-alignment-at-waypoint-a\|JA-115]] | Live LiDAR-Wall Relative Alignment at Waypoint A | candidate | — | — |
| [[JA-116-pre-allocated-pytorch-input-tensor-reuse\|JA-116]] | Pre-Allocated PyTorch Input Tensor Reuse | ✅ Implemented | — | — |
| [[JA-117-non-blocking-websockets-payload-compression\|JA-117]] | Non-Blocking WebSockets Payload Compression | ✅ Implemented | — | — |
| [[JA-118-spatio-temporal-cluster-merging-for-multi-scale\|JA-118]] | Spatio-Temporal Cluster Merging for Multi-Scale Blob Extraction | candidate | — | — |
| [[JA-119-adaptive-lidar-based-dynamic-scan-angle-filterin\|JA-119]] | Adaptive Lidar-Based Dynamic Scan Angle Filtering | candidate | — | — |
| [[JA-120-target-centric-kalman-filter-gate-for-websocket\|JA-120]] | Target-Centric Kalman Filter Gate for WebSocket Telemetry Bandwidth Reduction | candidate | — | — |
| [[JA-121-vectorized-centroid-global-transformation-via-ma\|JA-121]] | Vectorized Centroid Global Transformation via Matrix Dot Products | candidate | — | — |
| [[JA-122-dynamic-path-deceleration-via-predictive-time-to\|JA-122]] | Dynamic Path Deceleration via Predictive Time-to-Collision (TTC) Hysteresis | ✅ Implemented | — | — |
| [[JA-123-depth-range-adaptive-morphological-filtering\|JA-123]] | Depth-Range-Adaptive Morphological Filtering | candidate | — | — |
| [[JA-124-imu-aided-visual-odometry-ego-motion-projection\|JA-124]] | IMU-Aided Visual Odometry Ego-Motion Projection | candidate | — | — |
| [[JA-125-dynamic-potential-fields-for-path-clearance-marg\|JA-125]] | Dynamic Potential Fields for Path Clearance Margins in Narrow Corridor Navigation | candidate | — | — |
| [[JA-126-vectorized-depth-slicing-using-bounding-box-coor\|JA-126]] | Vectorized Depth Slicing using Bounding Box Coordinates | candidate | — | — |
| [[JA-127-websocket-frame-compression-bypass-for-small-pay\|JA-127]] | WebSocket Frame Compression Bypass for Small Payloads | candidate | — | — |
| [[JA-128-auto-calibrating-depth-threshold-offset-via-ambi\|JA-128]] | Auto-Calibrating Depth Threshold Offset via Ambient Lighting Reference | candidate | — | — |
| [[JA-129-multi-scale-temporal-windowing-for-motion-featur\|JA-129]] | Multi-Scale Temporal Windowing for Motion Feature Engineering | candidate | — | — |
| [[JA-130-predictive-corridor-yaw-alignment-during-bypass\|JA-130]] | Predictive Corridor Yaw Alignment during Bypass Strafing | candidate | — | — |
| [[JA-131-memory-mapped-shared-ring-buffer-for-web-server\|JA-131]] | Memory-Mapped Shared Ring Buffer for Web Server IPC | candidate | — | — |
| [[JA-132-pytorch-jit-optimization-via-tensorrt-compilatio\|JA-132]] | PyTorch JIT Optimization via TensorRT Compilation (Torch-TensorRT) | candidate | — | — |
| [[JA-133-dynamic-scaling-of-dbscan-search-radius-eps-for\|JA-133]] | Dynamic Scaling of DBSCAN Search Radius (Eps) for LiDAR Points | candidate | — | — |
| [[JA-134-imu-based-visual-centroid-stabilization-under-pi\|JA-134]] | IMU-Based Visual Centroid Stabilization under Pitch/Roll Vibrations | candidate | — | — |
| [[JA-135-trajectory-aware-predictive-safety-slowdown-gati\|JA-135]] | Trajectory-Aware Predictive Safety Slowdown Gating | candidate | — | — |
| [[JA-136-vectorized-frame-downsampling-via-slice-offsets\|JA-136]] | Vectorized Frame Downsampling via Slice Offsets | candidate | — | — |
| [[JA-137-pre-compiled-pytorch-modules-via-jit-tracing-tra\|JA-137]] | Pre-Compiled PyTorch Modules via JIT Tracing (Trace-Optimization) | ✅ Implemented | — | — |
| [[JA-138-vectorized-spatial-grid-binning-for-lidar-points\|JA-138]] | Vectorized Spatial Grid Binning for Lidar Points | candidate | — | — |
| [[JA-139-dynamic-camera-gain-auto-exposure-control-for-sh\|JA-139]] | Dynamic Camera Gain & Auto-Exposure Control for Shadow Adaptability | candidate | — | — |
| [[JA-140-multi-track-kalman-filter-prediction-for-mlp-win\|JA-140]] | Multi-Track Kalman Filter Prediction for MLP Window Alignment | candidate | — | — |
| [[JA-141-active-lateral-centering-potential-field-for-cor\|JA-141]] | Active Lateral Centering Potential Field for Corridor Navigation | ✅ Implemented | — | — |
| [[JA-142-gated-recurrent-unit-gru-temporal-feature-encodi\|JA-142]] | Gated Recurrent Unit (GRU) Temporal Feature Encoding | candidate | — | — |
| [[JA-143-dynamic-point-cloud-downsampling-with-adaptive-v\|JA-143]] | Dynamic Point Cloud Downsampling with Adaptive Voxel Grid Gating | ✅ Implemented | — | — |
| [[JA-144-lidar-scan-matching-for-relative-corridor-slip-e\|JA-144]] | LiDAR Scan Matching for Relative Corridor Slip Estimation | ✅ Implemented | — | — |
| [[JA-145-zero-copy-memory-mapped-frame-allocation-via-sha\|JA-145]] | Zero-Copy Memory-Mapped Frame Allocation via Shared Memory IPC | ✅ Implemented | — | — |
| [[JA-146-visual-lidar-geometric-depth-fusion-sensor-fusio\|JA-146]] | Visual-LiDAR Geometric Depth Fusion (Sensor Fusion Gating) | ✅ Implemented | — | — |
| [[JA-147-scan-match-corrected-ego-motion-compensation-for\|JA-147]] | Scan-Match Corrected Ego-Motion Compensation for Tracker History | candidate | — | — |
| [[JA-148-depth-gradient-watershed-segmentation-for-close\|JA-148]] | Depth Gradient Watershed Segmentation for Close-Proximity Obstacle Separation | candidate | — | — |
| [[JA-149-yaw-aware-dynamic-repulsion-clearance-based-rota\|JA-149]] | Yaw-Aware Dynamic Repulsion & Clearance-Based Rotation Center Shifting | candidate | — | — |
| [[JA-150-fast-1d-angular-index-search-for-lidar-scan-matc\|JA-150]] | Fast 1D Angular-Index Search for Lidar Scan Matching | candidate | — | — |
| [[JA-151-auto-calibrating-homography-projection-alignment\|JA-151]] | Auto-Calibrating Homography & Projection Alignment via YOLO Centroid Feedback | candidate | — | — |
| [[JA-152-physically-constrained-mlp-output-gating-kinemat\|JA-152]] | Physically Constrained MLP Output Gating (Kinematic Acceleration Bounding) | ✅ Implemented | — | — |
| [[JA-153-lidar-range-cluster-guided-depth-roi-extraction\|JA-153]] | LiDAR Range-Cluster Guided Depth ROI Extraction | candidate | — | — |
| [[JA-154-predictive-trajectory-intersector-bypassing-ttc\|JA-154]] | Predictive Trajectory-Intersector Bypassing (TTC-Guided Path Planning) | candidate | — | — |
| [[JA-155-vectorized-lidar-point-clustering-via-range-grad\|JA-155]] | Vectorized LiDAR Point Clustering via Range Gradient Masking | candidate | — | — |
| [[JA-156-real-time-control-jitter-and-energy-efficiency-m\|JA-156]] | Real-Time Control Jitter and Energy Efficiency Metrics Logger | candidate | — | — |
| [[JA-157-adaptively-adjusted-online-feature-normalization\|JA-157]] | Adaptively Adjusted Online Feature Normalization Gating | candidate | — | — |
| [[JA-158-lidar-depth-dynamic-extrinsics-auto-tuning-via-g\|JA-158]] | LiDAR-Depth Dynamic Extrinsics Auto-Tuning via Ground-Plane Invariance | candidate | — | — |
| [[JA-159-ransac-line-fitting-for-corridor-wall-boundary-i\|JA-159]] | RANSAC Line-Fitting for Corridor Wall Boundary Identification | candidate | — | — |
| [[JA-160-fast-path-gating-for-empty-track-states\|JA-160]] | Fast-Path Gating for Empty Track States | candidate | — | — |
| [[JA-161-bounding-box-temporal-prediction-gating-for-slow\|JA-161]] | Bounding Box Temporal Prediction Gating for Slow YOLO Framerates | candidate | — | — |
| [[JA-162-ego-velocity-weighted-feature-regularization\|JA-162]] | Ego-Velocity-Weighted Feature Regularization | candidate | — | — |
| [[JA-163-bilateral-depth-filtering-for-doorway-contour-pr\|JA-163]] | Bilateral Depth Filtering for Doorway Contour Preservation | candidate | — | — |
| [[JA-164-corridor-intersection-detection-speed-profiling\|JA-164]] | Corridor Intersection Detection & Speed Profiling via Angular LiDAR Entropy | candidate | — | — |
| [[JA-165-multi-threaded-frame-fetch-gating-producer-consu\|JA-165]] | Multi-Threaded Frame-Fetch Gating (Producer-Consumer Queue) | candidate | — | — |
| [[JA-166-dynamic-depth-segment-size-thresholding-based-on\|JA-166]] | Dynamic Depth Segment Size Thresholding based on Range | candidate | — | — |
| [[JA-167-relative-acceleration-input-features-for-interce\|JA-167]] | Relative Acceleration Input Features for Intercept Stability | candidate | — | — |
| [[JA-168-reflectivity-filtering-for-specular-ground-and-m\|JA-168]] | Reflectivity Filtering for Specular Ground and Metallic Surface Exclusions | candidate | — | — |
| [[JA-169-dynamic-corridor-angle-yaw-alignment-control\|JA-169]] | Dynamic Corridor Angle Yaw-Alignment Control | candidate | — | — |
| [[JA-170-direct-tensor-sharing-via-cuda-ipc-unified-gpu-m\|JA-170]] | Direct Tensor Sharing via CUDA IPC (Unified GPU Memory Pipeline) | candidate | — | — |
| [[JA-171-vectorized-lidar-based-yaw-drift-corrector\|JA-171]] | Vectorized LiDAR-based Yaw-Drift Corrector | candidate | — | — |
| [[JA-172-multi-modal-model-fusion-lidar-feature-combined\|JA-172]] | Multi-Modal Model Fusion (LiDAR-Feature Combined MLP) | candidate | — | — |
| [[JA-173-self-adapting-contrast-enhancement-local-clahe-f\|JA-173]] | Self-Adapting Contrast Enhancement (Local CLAHE) for Shadow Exclusions | candidate | — | — |
| [[JA-174-predictive-deceleration-profiling-for-dynamic-ob\|JA-174]] | Predictive Deceleration Profiling for Dynamic Obstacle Crossings | candidate | — | — |
| [[JA-175-vectorized-2d-rotation-using-numpy-multi-dimensi\|JA-175]] | Vectorized 2D Rotation using NumPy Multi-Dimensional Matrix Dot Products | candidate | — | — |
| [[JA-176-non-blocking-asynchronous-telemetry-logger\|JA-176]] | Non-Blocking Asynchronous Telemetry Logger | candidate | — | — |
| [[JA-177-multi-scale-temporal-feature-pooling-pyramid-tem\|JA-177]] | Multi-Scale Temporal Feature Pooling (Pyramid Temporal History Window) | candidate | — | — |
| [[JA-178-lidar-intensity-based-dynamic-floor-segment-filt\|JA-178]] | LiDAR Intensity-based Dynamic Floor Segment Filter | candidate | — | — |
| [[JA-179-proactive-deceleration-profiling-for-lateral-wal\|JA-179]] | Proactive Deceleration Profiling for Lateral Wall Clearances | candidate | — | — |
| [[JA-180-vectorized-lidar-corner-vertex-extraction-via-rd\|JA-180]] | Vectorized LiDAR Corner/Vertex Extraction via RDP Line Simplification | candidate | — | — |
| [[JA-181-double-buffered-shared-memory-ipc\|JA-181]] | Double-Buffered Shared Memory IPC | candidate | — | — |
| [[JA-182-track-frame-heading-normalization-for-mlp-genera\|JA-182]] | Track-Frame Heading Normalization for MLP Generalization | candidate | — | — |
| [[JA-183-k-means-guided-active-depth-cropping-roi-for-cro\|JA-183]] | K-Means guided Active Depth Cropping ROI for Crowded Scenarios | candidate | — | — |
| [[JA-184-corridor-cornering-clearance-compensation-via-sw\|JA-184]] | Corridor Cornering Clearance Compensation via Sweep Envelope Expansion | candidate | — | — |
| [[JA-185-vectorized-lidar-scan-compaction-via-adaptive-de\|JA-185]] | Vectorized LiDAR Scan Compaction via Adaptive Decimation | candidate | — | — |
| [[JA-186-temporal-tracking-gate-hysteresis-via-multi-fram\|JA-186]] | Temporal Tracking Gate Hysteresis via Multi-Frame Bounding Box Matching | candidate | — | — |
| [[JA-187-precomputed-lidar-angle-array-cache\|JA-187]] | Precomputed LiDAR Angle Array Cache | candidate | — | — |
| [[JA-188-laserscan-range-conversion-to-numpy-array\|JA-188]] | LaserScan Range Conversion to NumPy Array | candidate | — | — |
| [[JA-189-adaptive-icp-convergence-criterion-with-early-ex\|JA-189]] | Adaptive ICP Convergence Criterion with Early Exit | candidate | — | — |
| [[JA-190-icp-scan-match-inlier-quality-gate\|JA-190]] | ICP Scan Match Inlier Quality Gate | candidate | — | — |
| [[JA-191-speed-adaptive-forward-lidar-detection-range\|JA-191]] | Speed-Adaptive Forward LiDAR Detection Range | candidate | — | — |
| [[JA-192-twist-message-object-pre-allocation-in-control-l\|JA-192]] | Twist Message Object Pre-Allocation in Control Loop | candidate | — | — |
| [[JA-193-asymmetric-lateral-acceleration-and-deceleration\|JA-193]] | Asymmetric Lateral Acceleration and Deceleration Rate Limiting | candidate | — | — |
| [[JA-194-icp-correction-delta-bound-clamping-for-long-run\|JA-194]] | ICP Correction Delta Bound Clamping for Long-Run Stability | candidate | — | — |
| [[JA-195-mlp-prediction-variance-based-confidence-weighti\|JA-195]] | MLP Prediction Variance-Based Confidence Weighting for TTC Scaling | candidate | — | — |
| [[JA-196-dynamic-track-association-radius-based-on-last-e\|JA-196]] | Dynamic Track Association Radius Based on Last Estimated Speed | candidate | — | — |
| [[JA-197-ema-smoothed-wall-clearance-values-for-apf-jitte\|JA-197]] | EMA-Smoothed Wall Clearance Values for APF Jitter Prevention | ✅ Implemented | — | — |
| [[JA-198-icp-point-cloud-uniform-subsampling-for-cpu-effi\|JA-198]] | ICP Point Cloud Uniform Subsampling for CPU Efficiency | candidate | — | — |
| [[JA-199-repeat-mode-prev-scan-points-reset-to-prevent-cr\|JA-199]] | Repeat-Mode `prev_scan_points` Reset to Prevent Cross-Run ICP Corruption | candidate | — | — |
| [[JA-200-non-blocking-recovery-backing-via-timer-based-st\|JA-200]] | Non-Blocking Recovery Backing via Timer-Based State | ✅ Implemented | — | — |
| [[JA-201-visible-count-decay-on-unmatched-track-frames-in\|JA-201]] | `visible_count` Decay on Unmatched Track Frames in `ObstacleTracker` | candidate | — | — |
| [[JA-202-division-pre-inversion-for-faster-feature-normal\|JA-202]] | Division Pre-Inversion for Faster Feature Normalization | candidate | — | — |
| [[JA-203-multi-frame-confirmation-counter-for-front-is-co\|JA-203]] | Multi-Frame Confirmation Counter for `_front_is_continuous_wall` | ✅ Implemented | — | — |
| [[JA-204-per-track-distance-gate-before-mlp-inference-to\|JA-204]] | Per-Track Distance Gate Before MLP Inference to Skip Far Targets | candidate | — | — |
| [[JA-205-cross-track-error-and-commanded-velocity-columns\|JA-205]] | Cross-Track Error and Commanded Velocity Columns in RunLogger | candidate | — | — |
| [[JA-206-static-fixture-tagging-via-ema-motion-score-to-c\|JA-206]] | Static Fixture Tagging via EMA Motion Score to Clean Up TTC Loop | candidate | — | — |
| [[JA-207-settle-state-stop-command-deduplication-via-publ\|JA-207]] | Settle-State Stop Command Deduplication via Published-Flag | candidate | — | — |
| [[JA-208-adaptive-icp-iteration-count-based-on-commanded\|JA-208]] | Adaptive ICP Iteration Count Based on Commanded Speed | candidate | — | — |
| [[JA-209-cache-start-yaw-trigonometry-constants-at-segmen\|JA-209]] | Cache `start_yaw` Trigonometry Constants at Segment Transitions | ✅ Implemented | — | — |
| [[JA-210-depth-colorization-lazy-gating-by-client-subscri\|JA-210]] | Depth Colorization Lazy Gating by Client Subscription State | candidate | — | — |
| [[JA-211-single-estimates-snapshot-per-control-loop-tick\|JA-211]] | Single `_estimates` Snapshot Per Control Loop Tick | candidate | — | — |
| [[JA-212-store-prev-scan-points-directly-as-pre-converted\|JA-212]] | Store `prev_scan_points` Directly as Pre-Converted NumPy Array | candidate | — | — |
| [[JA-213-vectorized-continuous-wall-edge-detection-via-np\|JA-213]] | Vectorized Continuous Wall Edge Detection via `np.diff` on Front Sector | candidate | — | — |
| [[JA-214-apf-repulsion-saturation-to-prevent-anti-bypass\|JA-214]] | APF Repulsion Saturation to Prevent Anti-Bypass Interference | candidate | — | — |
| [[JA-215-bypass-clearance-confirmation-countdown-to-suppr\|JA-215]] | Bypass Clearance Confirmation Countdown to Suppress Re-Engagement Oscillation | candidate | — | — |
| [[JA-216-dual-depth-frame-acquisition-via-single-ros2brid\|JA-216]] | Dual Depth Frame Acquisition via Single `ROS2Bridge` Lock | candidate | — | — |
| [[JA-217-scale-vy-rep-proportionally-to-max-allowed-offse\|JA-217]] | Scale `vy_rep` Proportionally to `max_allowed_offset` in Tight Corridors | candidate | — | — |
| [[JA-218-z-score-outlier-rejection-for-centroid-depth-med\|JA-218]] | Z-Score Outlier Rejection for Centroid Depth Median Accuracy | candidate | — | — |
| [[JA-219-icp-warm-start-from-previous-residual-correction\|JA-219]] | ICP Warm-Start from Previous Residual Correction for Lateral Slip Compensation | candidate | — | — |
| [[JA-220-integral-cross-track-error-term-for-steady-state\|JA-220]] | Integral Cross-Track Error Term for Steady-State Lateral Drift Elimination | candidate | — | — |
| [[JA-221-icp-accumulated-rotation-delta-bound-clamping\|JA-221]] | ICP Accumulated Rotation Delta Bound Clamping | candidate | — | — |
| [[JA-222-vectorized-xy-scan-point-array-construction-from\|JA-222]] | Vectorized XY Scan-Point Array Construction from Cached Angle Array | candidate | — | — |
| [[JA-223-hard-physical-clamp-on-inverse-scaled-mlp-output\|JA-223]] | Hard Physical Clamp on Inverse-Scaled MLP Output Velocities | candidate | — | — |
| [[JA-224-forward-speed-ramp-up-delay-after-pause-release\|JA-224]] | Forward Speed Ramp-Up Delay After Pause Release | ✅ Implemented | — | — |
| [[JA-225-ekf-pose-staleness-guard-in-get-robot-pose-and-t\|JA-225]] | EKF Pose Staleness Guard in `get_robot_pose_and_twist` | ✅ Implemented | — | — |
| [[JA-226-icp-skip-during-high-lateral-command-velocity\|JA-226]] | ICP Skip During High Lateral Command Velocity | candidate | — | — |
| [[JA-227-remove-redundant-local-history-deque-from-obstac\|JA-227]] | Remove Redundant Local `history` Deque from `ObstacleTracker` | candidate | — | — |
| [[JA-228-minimum-range-safety-halt-during-rotate-states\|JA-228]] | Minimum-Range Safety Halt During ROTATE States | candidate | — | — |
| [[JA-229-icp-corrected-pose-columns-in-runlogger\|JA-229]] | ICP-Corrected Pose Columns in `RunLogger` | candidate | — | — |
| [[JA-230-depth-frame-staleness-gate-in-inference-loop\|JA-230]] | Depth Frame Staleness Gate in `_inference_loop` | ✅ Implemented | — | — |
| [[JA-231-apf-vy-rep-deadband-for-near-zero-jitter-elimina\|JA-231]] | APF `vy_rep` Deadband for Near-Zero Jitter Elimination | candidate | — | — |
| [[JA-232-segment-specific-rotation-timeout\|JA-232]] | Segment-Specific Rotation Timeout | candidate | — | — |
| [[JA-233-settle-state-icp-reference-scan-capture\|JA-233]] | Settle-State ICP Reference Scan Capture | candidate | — | — |
| [[JA-234-far-range-voxel-grid-creation-at-working-resolut\|JA-234]] | Far-Range Voxel Grid Creation at Working Resolution | candidate | — | — |
| [[JA-235-inference-loop-minimum-sleep-guard-for-cpu-overr\|JA-235]] | `_inference_loop` Minimum Sleep Guard for CPU Overrun Prevention | candidate | — | — |
| [[JA-236-visible-count-scaled-confidence-weight-on-mlp-ou\|JA-236]] | `visible_count`-Scaled Confidence Weight on MLP Output Speed | candidate | — | — |
| [[JA-237-scan-minimum-points-gate-increase-for-icp-reliab\|JA-237]] | Scan Minimum-Points Gate Increase for ICP Reliability | candidate | — | — |
| [[JA-238-per-obstacle-speed-ema-smoothing-for-ttc-stabili\|JA-238]] | Per-Obstacle Speed EMA Smoothing for TTC Stability | candidate | — | — |
| [[JA-239-blocked-time-counter-explicit-reset-on-drive-leg\|JA-239]] | Blocked-Time Counter Explicit Reset on Drive-Leg Transition | candidate | — | — |
| [[JA-240-lateral-p-gain-scheduling-based-on-proximity-to\|JA-240]] | Lateral P-Gain Scheduling Based on Proximity to Target Offset | candidate | — | — |
| [[JA-241-icp-computation-offloaded-to-a-dedicated-backgro\|JA-241]] | ICP Computation Offloaded to a Dedicated Background Thread | candidate | — | — |
| [[JA-242-forward-sector-lidar-beam-count-gate-for-wall-vs\|JA-242]] | Forward-Sector LiDAR Beam Count Gate for Wall-vs-Obstacle Classification | candidate | — | — |
| [[JA-243-obstacletracker-history-deque-reset-at-repeat-mo\|JA-243]] | ObstacleTracker History Deque Reset at Repeat-Mode Run Boundaries | candidate | — | — |
| [[JA-244-round-trip-pose-drift-magnitude-logged-at-run-co\|JA-244]] | Round-Trip Pose Drift Magnitude Logged at Run Completion | candidate | — | — |
| [[JA-245-ema-smoothing-on-min-forward-lidar-to-prevent-sp\|JA-245]] | EMA Smoothing on `_min_forward_lidar` to Prevent Specular False Early-Stops | candidate | — | — |
| [[JA-246-icp-pose-corruption-detection-via-negative-dist\|JA-246]] | ICP Pose Corruption Detection via Negative `dist_error` Monitor | candidate | — | — |
| [[JA-247-is-paused-column-in-runlogger-for-drive-vs-wait\|JA-247]] | `is_paused` Column in RunLogger for Drive-vs-Wait Segment Analysis | candidate | — | — |
| [[JA-248-dynamic-obstacletracker-max-age-scaling-based-on\|JA-248]] | Dynamic `ObstacleTracker.max_age` Scaling Based on Estimation Mode | candidate | — | — |
| [[JA-249-settle-state-physical-stop-confirmation-via-corr\|JA-249]] | Settle-State Physical Stop Confirmation via Corrected-Pose Delta Gate | candidate | — | — |
| [[JA-250-depth-centroid-vertical-position-gate-to-reject\|JA-250]] | Depth Centroid Vertical Position Gate to Reject Floor-Level Ghost Detections | candidate | — | — |
| [[JA-251-cached-cos-r-sin-r-with-change-threshold-refresh\|JA-251]] | Cached `cos_r`/`sin_r` with Change-Threshold Refresh in `_inference_loop` | candidate | — | — |
| [[JA-252-binary-websocket-message-pre-filter-in-ab-test-e\|JA-252]] | Binary WebSocket Message Pre-Filter in AB Test Estimator Receive Loop | candidate | — | — |
| [[JA-253-forward-scale-bypass-denominator-tied-to-active\|JA-253]] | Forward-Scale Bypass Denominator Tied to Active `BYPASS_OFFSET` | candidate | — | — |
| [[JA-254-redundant-get-speed-scaling-call-elimination-in\|JA-254]] | Redundant `_get_speed_scaling()` Call Elimination in Diagnostic Block | candidate | — | — |
| [[JA-255-scan-dirty-flag-to-skip-bypass-sector-computatio\|JA-255]] | Scan-Dirty Flag to Skip Bypass Sector Computation Between LiDAR Updates | candidate | — | — |
| [[JA-256-blocked-time-column-in-runlogger-for-recovery-ev\|JA-256]] | `blocked_time` Column in RunLogger for Recovery Event Traceability | candidate | — | — |
| [[JA-257-icp-skip-gate-based-on-odom-delta-magnitude-for\|JA-257]] | ICP Skip Gate Based on Odom Delta Magnitude for Stationary Phases | candidate | — | — |
| [[JA-258-pre-computed-apf-normalization-constant-to-elimi\|JA-258]] | Pre-Computed APF Normalization Constant to Eliminate Per-Tick Divisions | candidate | — | — |
| [[JA-259-detections-fn-hoisted-outside-per-contour-loop-f\|JA-259]] | `detections_fn()` Hoisted Outside Per-Contour Loop for Gating Efficiency | candidate | — | — |
| [[JA-260-quadratic-deceleration-profile-in-final-0-30-m-a\|JA-260]] | Quadratic Deceleration Profile in Final 0.30 m Approach Zone | candidate | — | — |
| [[JA-261-time-based-track-age-expiry-for-throttle-robust\|JA-261]] | Time-Based Track Age Expiry for Throttle-Robust Track Persistence | candidate | — | — |
| [[JA-262-runlogger-incremental-file-write-for-repeat-mode\|JA-262]] | RunLogger Incremental File Write for Repeat-Mode Memory Reduction | candidate | — | — |
| [[JA-263-normalize-angle-o-1-modulo-formula-replacement\|JA-263]] | `normalize_angle` O(1) Modulo Formula Replacement | candidate | — | — |
| [[JA-264-min-forward-lidar-column-in-runlogger-for-wall-p\|JA-264]] | `_min_forward_lidar` Column in RunLogger for Wall-Proximity Analysis | candidate | — | — |
| [[JA-265-per-segment-rms-cross-track-error-summary-row-in\|JA-265]] | Per-Segment RMS Cross-Track Error Summary Row in RunLogger | candidate | — | — |
| [[JA-266-adaptive-apf-repulsion-eta-scaled-by-current-for\|JA-266]] | Adaptive APF Repulsion `eta` Scaled by Current Forward Speed | candidate | — | — |
| [[JA-267-waypoint-arrival-hysteresis-band-to-prevent-icp\|JA-267]] | Waypoint Arrival Hysteresis Band to Prevent ICP-Noise False Arrivals | candidate | — | — |
| [[JA-268-n-icp-inliers-column-in-runlogger-for-scan-match\|JA-268]] | `n_icp_inliers` Column in RunLogger for Scan Match Quality Monitoring | candidate | — | — |
| [[JA-269-front-has-edges-minimum-edge-count-gate-for-spec\|JA-269]] | `front_has_edges` Minimum Edge-Count Gate for Specular-Robust Wall Classification | candidate | — | — |
| [[JA-270-speed-scale-ema-minimum-per-tick-recovery-increm\|JA-270]] | Speed-Scale EMA Minimum Per-Tick Recovery Increment | candidate | — | — |
| [[JA-271-state-transition-marker-rows-in-runlogger-for-un\|JA-271]] | State Transition Marker Rows in RunLogger for Unambiguous Segment Boundaries | candidate | — | — |
| [[JA-272-stop-robot-reduced-from-3-publish-loop-to-single\|JA-272]] | `_stop_robot` Reduced from 3-Publish Loop to Single Publish | candidate | — | — |
| [[JA-273-icp-disabled-during-rotation-and-settle-states\|JA-273]] | ICP Disabled During Rotation and Settle States | candidate | — | — |
| [[JA-274-corrected-yaw-soft-re-sync-to-ekf-heading-at-dri\|JA-274]] | `corrected_yaw` Soft Re-Sync to EKF Heading at Drive-Segment Entry | candidate | — | — |
| [[JA-275-speed-proportional-wall-stop-dist-lookahead-buff\|JA-275]] | Speed-Proportional `WALL_STOP_DIST` Lookahead Buffer | candidate | — | — |
| [[JA-276-bypass-direction-selection-via-pedestrian-latera\|JA-276]] | Bypass Direction Selection via Pedestrian Lateral Velocity Projection | candidate | — | — |
| [[JA-277-yolo-person-detection-veto-for-is-continuous-wal\|JA-277]] | YOLO Person Detection Veto for `is_continuous_wall` Stationary Pedestrian Case | candidate | — | — |
| [[JA-278-lidar-range-rate-forward-sector-ttc-supplement-f\|JA-278]] | LiDAR Range-Rate Forward Sector TTC Supplement for Reactive Mode | candidate | — | — |
| [[JA-279-last-vy-cmd-reset-after-recovery-backing-maneuve\|JA-279]] | `last_vy_cmd` Reset After Recovery Backing Maneuver | candidate | — | — |
| [[JA-280-is-dynamic-pedestrian-hysteresis-latch-to-preven\|JA-280]] | `is_dynamic_pedestrian` Hysteresis Latch to Prevent Mid-Bypass Corridor Narrowing | candidate | — | — |
| [[JA-281-approach-velocity-scaled-avoidance-forward-for-e\|JA-281]] | Approach-Velocity-Scaled `AVOIDANCE_FORWARD` for Earlier Bypass Initiation | candidate | — | — |
| [[JA-282-wall-stop-and-bypass-suppression-event-counters\|JA-282]] | Wall-Stop and Bypass-Suppression Event Counters in RunLogger | candidate | — | — |
| [[JA-283-predictive-bypass-pre-initiation-via-forward-pro\|JA-283]] | Predictive Bypass Pre-Initiation via Forward-Projected Pedestrian Position | candidate | — | — |
| [[JA-284-range-adaptive-alpha-z-ema-filter-based-on-centr\|JA-284]] | Range-Adaptive `alpha_z` EMA Filter Based on Centroid Depth Bin | candidate | — | — |
| [[JA-285-odom-body-velocity-extraction-for-settle-state-k\|JA-285]] | Odom Body Velocity Extraction for Settle-State Kinematic Confirmation | candidate | — | — |
| [[JA-286-bypass-direction-lock-in-timer-to-prevent-flip-o\|JA-286]] | Bypass Direction Lock-In Timer to Prevent Flip Oscillation | candidate | — | — |
| [[JA-287-hist-local-forward-component-clamping-in-inferen\|JA-287]] | `hist_local` Forward-Component Clamping in `_inference_loop` | candidate | — | — |
| [[JA-288-icp-rotation-accumulator-re-orthogonalization-vi\|JA-288]] | ICP Rotation Accumulator Re-Orthogonalization via SVD Polar Factor | candidate | — | — |
| [[JA-289-per-track-mlp-output-vector-ema-for-stable-bypas\|JA-289]] | Per-Track MLP Output Vector EMA for Stable Bypass Direction Commands | candidate | — | — |
| [[JA-290-encode-mapu-offload-to-thread-executor-for-non-b\|JA-290]] | `_encode_mapu` Offload to Thread Executor for Non-Blocking Live Map Requests | candidate | — | — |
| [[JA-291-scan-callback-timestamp-staleness-guard-for-bypa\|JA-291]] | Scan Callback Timestamp Staleness Guard for Bypass Logic | candidate | — | — |
| [[JA-292-forward-speed-proportional-kp-yaw-scaling-to-pre\|JA-292]] | Forward-Speed Proportional `KP_YAW` Scaling to Prevent Near-Waypoint Yaw Oscillation | candidate | — | — |
| [[JA-293-pre-bypass-diagonal-sector-clearance-scan\|JA-293]] | Pre-Bypass Diagonal Sector Clearance Scan | candidate | — | — |
| [[JA-294-centroid-depth-standard-deviation-quality-gate-i\|JA-294]] | Centroid Depth Standard Deviation Quality Gate in `_extract_depth_centroids` | candidate | — | — |
| [[JA-295-speed-adaptive-icp-vs-odometry-ratio-plausibilit\|JA-295]] | Speed-Adaptive ICP vs. Odometry Ratio Plausibility Gate | candidate | — | — |
| [[JA-296-minimum-forward-speed-floor-to-prevent-stall-dur\|JA-296]] | Minimum Forward Speed Floor to Prevent Stall During Active Lateral Bypass | candidate | — | — |
| [[JA-297-reactive-mode-ttc-block-short-circuit-in-get-spe\|JA-297]] | Reactive-Mode TTC Block Short-Circuit in `_get_speed_scaling` | candidate | — | — |
| [[JA-298-rot-correction-attenuation-during-active-lateral\|JA-298]] | `rot_correction` Attenuation During Active Lateral Strafing | candidate | — | — |
| [[JA-299-websocket-estimate-timestamp-staleness-guard-in\|JA-299]] | WebSocket Estimate Timestamp Staleness Guard in AB Test Receive Loop | candidate | — | — |
| [[JA-300-icp-input-point-cloud-far-range-pre-filter-for-c\|JA-300]] | ICP Input Point Cloud Far-Range Pre-Filter for Correspondence Matrix Reduction | candidate | — | — |
| [[JA-301-lateral-threshold-dynamic-expansion-during-activ\|JA-301]] | `LATERAL_THRESHOLD` Dynamic Expansion During Active Bypass in `_get_speed_scaling` | candidate | — | — |
| [[JA-302-init-state-odometry-body-velocity-zero-confirmat\|JA-302]] | INIT-State Odometry Body Velocity Zero-Confirmation Before Reference Pose Capture | candidate | — | — |
| [[JA-303-pre-computed-morphological-kernel-in-velocityest\|JA-303]] | Pre-Computed Morphological Kernel in `VelocityEstimator.__init__` | candidate | — | — |
| [[JA-304-scan-subscription-qos-depth-1-best-effort-in-ab\|JA-304]] | `/scan` Subscription QoS `depth=1` `BEST_EFFORT` in `ab_comparison_test.py` | candidate | — | — |
| [[JA-305-last-vy-cmd-zero-reset-on-target-lateral-offset\|JA-305]] | `last_vy_cmd` Zero-Reset on `target_lateral_offset` Sign Flip for Faster Direction Reversal | candidate | — | — |
| [[JA-306-apf-d-safe-activation-hysteresis-band-to-elimina\|JA-306]] | APF `d_safe` Activation Hysteresis Band to Eliminate Near-Threshold Lateral Chatter | candidate | — | — |
| [[JA-307-icp-inlier-fraction-weighted-blend-with-raw-odom\|JA-307]] | ICP Inlier-Fraction Weighted Blend with Raw Odometry for Graceful Quality Transition | candidate | — | — |
| [[JA-308-self-current-path-y-sync-from-local-path-y-in-dr\|JA-308]] | `self.current_path_y` Sync from Local `path_y` in Drive States | candidate | — | — |
| [[JA-309-consecutive-icp-failure-counter-with-corrected-p\|JA-309]] | Consecutive ICP Failure Counter with Corrected-Pose EKF Re-Sync Trigger | candidate | — | — |
| [[JA-310-temporal-lidar-range-difference-gate-for-dynamic\|JA-310]] | Temporal LiDAR Range-Difference Gate for Dynamic Obstacle Confirmation in Forward Sector | candidate | — | — |
| [[JA-311-time-monotonic-parameter-injection-from-control\|JA-311]] | `time.monotonic()` Parameter Injection from `_control_loop` into `_update_bypass_offset` | candidate | — | — |
| [[JA-312-o-n-history-padding-in-build-window-features-rep\|JA-312]] | O(N²) History Padding in `_build_window_features` Replaced with List Concatenation | candidate | — | — |
| [[JA-313-odom-subscription-qos-depth-1-best-effort-for-st\|JA-313]] | `/odom` Subscription QoS `depth=1` `BEST_EFFORT` for Stale-Burst ICP Prevention | candidate | — | — |
| [[JA-314-icp-final-iteration-q-trans-update-skip-for-hot\|JA-314]] | ICP Final-Iteration `Q_trans` Update Skip for Hot-Path Reduction | candidate | — | — |
| [[JA-315-is-dynamic-pedestrian-result-reuse-in-continuous\|JA-315]] | `is_dynamic_pedestrian` Result Reuse in Continuous-Wall Filter to Eliminate Double Loop | candidate | — | — |
| [[JA-316-clear-path-lidar-second-scan-elimination-via-pre\|JA-316]] | Clear-Path LiDAR Second-Scan Elimination via Pre-Cached Sector Vectors | candidate | — | — |
| [[JA-317-dynamic-kp-lateral-gain-reduction-in-tight-corri\|JA-317]] | Dynamic `KP_LATERAL` Gain Reduction in Tight Corridor Mode | candidate | — | — |
| [[JA-318-apf-local-constants-elevated-to-abcomparisontest\|JA-318]] | APF Local Constants Elevated to `ABComparisonTest.__init__` for Hot-Path Bytecode Reduction | candidate | — | — |
| [[JA-319-maybe-log-obstacle-distance-vectorized-via-np-hy\|JA-319]] | `_maybe_log` Obstacle Distance Vectorized via `np.hypot` | candidate | — | — |
| [[JA-320-mlp-inference-warm-up-dummy-pass-in-load-model-t\|JA-320]] | MLP Inference Warm-Up Dummy Pass in `_load_model` to Eliminate First-Tick JIT Latency | candidate | — | — |
| [[JA-321-lateral-threshold-centered-on-active-bypass-offs\|JA-321]] | `LATERAL_THRESHOLD` Centered on Active Bypass Offset in `_get_speed_scaling` | candidate | — | — |
| [[JA-322-lidar-blocked-consecutive-frame-hysteresis-befor\|JA-322]] | `lidar_blocked` Consecutive-Frame Hysteresis Before Setting `is_paused` | candidate | — | — |
| [[JA-323-pre-allocated-normalization-output-buffer-to-eli\|JA-323]] | Pre-Allocated Normalization Output Buffer to Eliminate Per-Cycle Heap Allocations in `_inference_loop` | candidate | — | — |
| [[JA-324-smooth-speed-scale-reset-in-repeat-mode-rotate-h\|JA-324]] | `smooth_speed_scale` Reset in Repeat-Mode `ROTATE_HOME` Restart Block | candidate | — | — |
| [[JA-325-closest-obstacle-track-id-column-in-maybe-log-fo\|JA-325]] | Closest-Obstacle Track ID Column in `_maybe_log` for Post-Run Track Attribution | candidate | — | — |
| [[JA-326-np-vstack-eliminated-via-direct-per-track-write\|JA-326]] | `np.vstack` Eliminated via Direct Per-Track Write into Pre-Allocated Batch Feature Matrix | candidate | — | — |
| [[JA-327-depth-range-confidence-downscale-for-far-tracks\|JA-327]] | Depth-Range Confidence Downscale for Far Tracks (Z > 2.5 m) in `_get_speed_scaling` | candidate | — | — |
| [[JA-328-weighted-icp-correspondence-via-range-proportion\|JA-328]] | Weighted ICP Correspondence via Range-Proportional Point Weights in `_align_scans_icp` | candidate | — | — |
| [[JA-329-icp-vs-ekf-pose-discrepancy-real-time-warning-in\|JA-329]] | ICP vs. EKF Pose Discrepancy Real-Time Warning in 2 Hz Diagnostic Print | candidate | — | — |
| [[JA-330-global-history-upstream-displacement-clamping-be\|JA-330]] | Global History Upstream Displacement Clamping Before Local Frame Reconstruction in `_build_window_features` | candidate | — | — |
| [[JA-331-lidar-side-sector-wall-clearance-5th-percentile\|JA-331]] | LiDAR Side-Sector Wall Clearance 5th Percentile for Specular-Return Outlier Rejection | candidate | — | — |
| [[JA-332-scaler-parameters-null-guard-binding-to-model-di\|JA-332]] | Scaler Parameters Null-Guard Binding to Model Disable on JSON Load Failure | candidate | — | — |
| [[JA-333-icp-convergence-check-squared-norm-to-eliminate\|JA-333]] | ICP Convergence Check Squared-Norm to Eliminate `abs()` Function Dispatch | candidate | — | — |
| [[JA-334-set-estimator-mode-exponential-backoff-on-websoc\|JA-334]] | `_set_estimator_mode` Exponential Backoff on WebSocket Reconnection | candidate | — | — |
| [[JA-335-forward-lidar-blockage-cone-narrowed-from-30-to\|JA-335]] | Forward LiDAR Blockage Cone Narrowed from ±30° to ±20° for `lidar_blocked` Check | candidate | — | — |
| [[JA-336-icp-cross-covariance-matrix-ill-conditioning-gua\|JA-336]] | ICP Cross-Covariance Matrix Ill-Conditioning Guard for Collinear Wall Scans | candidate | — | — |
| [[JA-337-dead-est-get-y-0-0-fallback-removal-from-depth-c\|JA-337]] | Dead `est.get('y', 0.0)` Fallback Removal from Depth Coordinate Reads in AB Test | candidate | — | — |
| [[JA-338-per-track-feature-construction-buffer-pre-alloca\|JA-338]] | Per-Track Feature Construction Buffer Pre-Allocation in `_build_window_features` | candidate | — | — |
| [[JA-339-obstacletracker-max-dist-reduction-in-reactive-m\|JA-339]] | `ObstacleTracker.max_dist` Reduction in Reactive Mode to Prevent False ID Merges | candidate | — | — |
| [[JA-340-depth-continuity-guard-in-obstacletracker-update\|JA-340]] | Depth Continuity Guard in `ObstacleTracker.update()` to Prevent Background Contamination | candidate | — | — |

## June — Performance (`JP-`)

120 ideas, 0 implemented — source: `ideas/June_performance_ideas.md`

| ID | Title | Status | ROI | Domain |
| :-- | :-- | :-- | :-- | :-- |
| [[JP-001-optimize-depth-map-downsampling\|JP-001]] | Optimize Depth Map Downsampling | candidate | — | — |
| [[JP-002-move-depth-colormapping-to-the-client\|JP-002]] | Move Depth Colormapping to the Client | candidate | — | — |
| [[JP-003-hardware-acceleration-for-models-tensorrt\|JP-003]] | Hardware Acceleration for Models (TensorRT) | candidate | — | — |
| [[JP-004-decouple-dom-updates-in-the-gui\|JP-004]] | Decouple DOM Updates in the GUI | candidate | — | — |
| [[JP-005-fast-squared-distance-calculation-for-tracking\|JP-005]] | Fast Squared Distance Calculation for Tracking | candidate | — | — |
| [[JP-006-remove-blocking-print-statements\|JP-006]] | Remove Blocking Print Statements | candidate | — | — |
| [[JP-007-optimize-canvas-rendering-composition\|JP-007]] | Optimize Canvas Rendering Composition | candidate | — | — |
| [[JP-008-replace-np-median-with-np-mean-for-faster-profil\|JP-008]] | Replace np.median with np.mean for Faster Profiling | candidate | — | — |
| [[JP-009-optimize-mask-allocations-with-pre-allocation\|JP-009]] | Optimize Mask Allocations with Pre-allocation | candidate | — | — |
| [[JP-010-avoid-global-python-locks-for-reference-swapping\|JP-010]] | Avoid Global Python Locks for Reference Swapping | candidate | — | — |
| [[JP-011-isolate-yolo-inference-into-a-subprocess\|JP-011]] | Isolate YOLO Inference into a Subprocess | candidate | — | — |
| [[JP-012-externalize-inline-svg-assets\|JP-012]] | Externalize Inline SVG Assets | candidate | — | — |
| [[JP-013-skip-mask-arrays-for-fast-depth-slicing\|JP-013]] | Skip Mask Arrays for Fast Depth Slicing | candidate | — | — |
| [[JP-014-ensure-native-rust-json-serialization\|JP-014]] | Ensure Native Rust JSON Serialization | candidate | — | — |
| [[JP-015-throttle-frontend-command-submissions\|JP-015]] | Throttle Frontend Command Submissions | candidate | — | — |
| [[JP-016-eliminate-tiny-matrix-instantiation-overhead\|JP-016]] | Eliminate Tiny Matrix Instantiation Overhead | candidate | — | — |
| [[JP-017-skip-neural-inference-for-missing-tracks\|JP-017]] | Skip Neural Inference for Missing Tracks | candidate | — | — |
| [[JP-018-hardware-accelerated-video-encoding\|JP-018]] | Hardware-Accelerated Video Encoding | candidate | — | — |
| [[JP-019-prevent-font-rendering-jitter-in-telemetry\|JP-019]] | Prevent Font Rendering Jitter in Telemetry | candidate | — | — |
| [[JP-020-upgrade-to-event-driven-spin-waits\|JP-020]] | Upgrade to Event-Driven Spin-Waits | candidate | — | — |
| [[JP-021-vectorize-hungarian-matching-in-tracking\|JP-021]] | Vectorize Hungarian Matching in Tracking | candidate | — | — |
| [[JP-022-consolidate-web-server-with-async-framework\|JP-022]] | Consolidate Web Server with Async Framework | candidate | — | — |
| [[JP-023-cap-pytorch-thread-count-for-small-mlps\|JP-023]] | Cap PyTorch Thread Count for Small MLPs | candidate | — | — |
| [[JP-024-batch-dom-updates-via-virtual-dom-principles\|JP-024]] | Batch DOM Updates via Virtual DOM Principles | candidate | — | — |
| [[JP-025-replace-opencv-morphology-with-statistical-outli\|JP-025]] | Replace OpenCV Morphology with Statistical Outlier Removal | candidate | — | — |
| [[JP-026-use-simd-optimized-cv2-inrange-for-depth-ranges\|JP-026]] | Use SIMD-Optimized cv2.inRange for Depth Ranges | candidate | — | — |
| [[JP-027-fully-local-offline-cdn-assets\|JP-027]] | Fully Local Offline CDN Assets | candidate | — | — |
| [[JP-028-refactor-subprocess-spawning-to-native-asyncio\|JP-028]] | Refactor Subprocess Spawning to Native Asyncio | candidate | — | — |
| [[JP-029-defer-math-processing-via-lazy-evaluation\|JP-029]] | Defer Math Processing via Lazy Evaluation | candidate | — | — |
| [[JP-030-vectorized-neural-post-processing\|JP-030]] | Vectorized Neural Post-Processing | candidate | — | — |
| [[JP-031-render-throttling-for-stationary-status\|JP-031]] | Render Throttling for Stationary Status | candidate | — | — |
| [[JP-032-eliminate-thread-polling-via-async-event-listene\|JP-032]] | Eliminate Thread Polling via Async Event Listeners | candidate | — | — |
| [[JP-033-pre-encode-telemetry-for-client-broadcasting\|JP-033]] | Pre-Encode Telemetry for Client Broadcasting | candidate | — | — |
| [[JP-034-aggressive-pytorch-jit-optimization\|JP-034]] | Aggressive PyTorch JIT Optimization | candidate | — | — |
| [[JP-035-force-garbage-collection-during-model-swaps\|JP-035]] | Force Garbage Collection During Model Swaps | candidate | — | — |
| [[JP-036-remove-heavy-gpu-css-rasterization\|JP-036]] | Remove Heavy GPU CSS Rasterization | candidate | — | — |
| [[JP-037-integral-image-pre-calculation-for-depth\|JP-037]] | Integral Image Pre-calculation for Depth | candidate | — | — |
| [[JP-038-use-lazy-logging-to-avoid-string-overhead\|JP-038]] | Use Lazy Logging to Avoid String Overhead | candidate | — | — |
| [[JP-039-switch-websockets-from-base64-to-arraybuffers\|JP-039]] | Switch WebSockets from Base64 to ArrayBuffers | candidate | — | — |
| [[JP-040-ring-buffers-instead-of-python-deques\|JP-040]] | Ring Buffers Instead of Python Deques | candidate | — | — |
| [[JP-041-zero-copy-tensor-initialization\|JP-041]] | Zero-Copy Tensor Initialization | candidate | — | — |
| [[JP-042-enforce-hardware-camera-capture-resolution\|JP-042]] | Enforce Hardware Camera Capture Resolution | candidate | — | — |
| [[JP-043-async-map-decodes-via-createimagebitmap\|JP-043]] | Async Map Decodes via createImageBitmap | candidate | — | — |
| [[JP-044-early-exit-low-confidence-detections\|JP-044]] | Early-Exit Low Confidence Detections | candidate | — | — |
| [[JP-045-guarantee-memory-contiguity-for-box-crops\|JP-045]] | Guarantee Memory Contiguity for Box Crops | candidate | — | — |
| [[JP-046-manual-deterministic-garbage-collection\|JP-046]] | Manual Deterministic Garbage Collection | candidate | — | — |
| [[JP-047-typed-arrays-for-bulk-frontend-data\|JP-047]] | Typed Arrays for Bulk Frontend Data | candidate | — | — |
| [[JP-048-strip-heavy-covariance-matrices-from-odometry\|JP-048]] | Strip Heavy Covariance Matrices from Odometry | candidate | — | — |
| [[JP-049-downsample-lidar-points-for-cbf-solver\|JP-049]] | Downsample Lidar Points for CBF Solver | candidate | — | — |
| [[JP-050-cache-trigonometric-lookup-table-for-lidar\|JP-050]] | Cache Trigonometric Lookup Table for Lidar | candidate | — | — |
| [[JP-051-shared-memory-for-depth-frames-to-velocity-estim\|JP-051]] | Shared Memory for Depth Frames to Velocity Estimator | candidate | — | — |
| [[JP-052-debounce-rapid-websocket-state-changes\|JP-052]] | Debounce Rapid WebSocket State Changes | candidate | — | — |
| [[JP-053-vectorize-lidar-polar-to-cartesian-in-scan-cb\|JP-053]] | Vectorize Lidar Polar-to-Cartesian in _scan_cb | candidate | — | — |
| [[JP-054-conditional-yolo-inference-skip-on-idle\|JP-054]] | Conditional YOLO Inference Skip on Idle | candidate | — | — |
| [[JP-055-use-css-content-visibility-auto-for-off-screen-c\|JP-055]] | Use CSS `content-visibility: auto` for Off-Screen Cards | candidate | — | — |
| [[JP-056-profile-guided-cbf-gamma-tuning\|JP-056]] | Profile-Guided CBF Gamma Tuning | candidate | — | — |
| [[JP-057-replace-scipy-slsqp-with-closed-form-cbf-project\|JP-057]] | Replace scipy SLSQP with Closed-Form CBF Projection | candidate | — | — |
| [[JP-058-compress-websocket-camera-frames-with-webp\|JP-058]] | Compress WebSocket Camera Frames with WebP | candidate | — | — |
| [[JP-059-lazy-import-heavy-modules-in-ros-callbacks\|JP-059]] | Lazy-Import Heavy Modules in ROS Callbacks | candidate | — | — |
| [[JP-060-use-struct-pack-for-binary-telemetry-instead-of\|JP-060]] | Use `struct.pack` for Binary Telemetry Instead of JSON | candidate | — | — |
| [[JP-061-numpy-vectorized-scan-cb-with-boolean-masking\|JP-061]] | NumPy Vectorized _scan_cb with Boolean Masking | candidate | — | — |
| [[JP-062-deduplicate-depth-subscription-callbacks\|JP-062]] | Deduplicate Depth Subscription Callbacks | candidate | — | — |
| [[JP-063-offload-three-js-rendering-to-offscreencanvas\|JP-063]] | Offload Three.js Rendering to OffscreenCanvas | candidate | — | — |
| [[JP-064-use-np-empty-instead-of-np-zeros-for-scratch-buf\|JP-064]] | Use `np.empty` Instead of `np.zeros` for Scratch Buffers | candidate | — | — |
| [[JP-065-connection-aware-broadcast-frequency\|JP-065]] | Connection-Aware Broadcast Frequency | candidate | — | — |
| [[JP-066-fuse-cbf-filter-directly-into-motion-loop\|JP-066]] | Fuse CBF Filter Directly into motion_loop | candidate | — | — |
| [[JP-067-reduce-occupancy-grid-serialization-with-run-len\|JP-067]] | Reduce Occupancy Grid Serialization with Run-Length Encoding | candidate | — | — |
| [[JP-068-eliminate-redundant-dict-copies-in-getters\|JP-068]] | Eliminate Redundant `dict()` Copies in Getters | candidate | — | — |
| [[JP-069-warm-start-the-cbf-solver\|JP-069]] | Warm-Start the CBF Solver | candidate | — | — |
| [[JP-070-use-cv2-inter-nearest-for-non-display-resizes\|JP-070]] | Use `cv2.INTER_NEAREST` for Non-Display Resizes | candidate | — | — |
| [[JP-071-service-worker-cache-for-static-gui-assets\|JP-071]] | Service Worker Cache for Static GUI Assets | candidate | — | — |
| [[JP-072-batch-multiple-ros-callbacks-with-a-single-lock\|JP-072]] | Batch Multiple ROS Callbacks with a Single Lock Acquisition | candidate | — | — |
| [[JP-073-onnx-runtime-for-velocity-mlp-instead-of-pytorch\|JP-073]] | ONNX Runtime for Velocity MLP Instead of PyTorch | candidate | — | — |
| [[JP-074-adaptive-broadcast-rate-based-on-network-latency\|JP-074]] | Adaptive Broadcast Rate Based on Network Latency | candidate | — | — |
| [[JP-075-inline-critical-css-and-defer-non-critical-style\|JP-075]] | Inline Critical CSS and Defer Non-Critical Styles | candidate | — | — |
| [[JP-076-spatial-hashing-for-cbf-obstacle-lookup\|JP-076]] | Spatial Hashing for CBF Obstacle Lookup | candidate | — | — |
| [[JP-077-guard-cbf-with-velocity-magnitude-check\|JP-077]] | Guard CBF with Velocity Magnitude Check | candidate | — | — |
| [[JP-078-use-itertools-compress-for-sparse-detection-filt\|JP-078]] | Use `itertools.compress` for Sparse Detection Filtering | candidate | — | — |
| [[JP-079-websocket-heartbeat-pings-for-stale-client-pruni\|JP-079]] | WebSocket Heartbeat Pings for Stale Client Pruning | candidate | — | — |
| [[JP-080-quantize-depth-frame-to-uint16-before-processing\|JP-080]] | Quantize Depth Frame to uint16 Before Processing | candidate | — | — |
| [[JP-081-closed-form-half-plane-cbf-projection\|JP-081]] | Closed-Form Half-Plane CBF Projection | candidate | — | — |
| [[JP-082-preallocate-websocket-send-buffers\|JP-082]] | Preallocate WebSocket Send Buffers | candidate | — | — |
| [[JP-083-gpu-accelerated-depth-colormap-via-cuda\|JP-083]] | GPU-Accelerated Depth Colormap via CUDA | candidate | — | — |
| [[JP-084-eliminate-redundant-float-casts-in-move\|JP-084]] | Eliminate Redundant `float()` Casts in move() | candidate | — | — |
| [[JP-085-temporal-smoothing-for-cbf-output\|JP-085]] | Temporal Smoothing for CBF Output | candidate | — | — |
| [[JP-086-avoid-re-encoding-unchanged-camera-frames\|JP-086]] | Avoid Re-encoding Unchanged Camera Frames | candidate | — | — |
| [[JP-087-replace-python-list-append-loop-with-numpy-stack\|JP-087]] | Replace Python `list.append` Loop with NumPy `stack` | candidate | — | — |
| [[JP-088-compress-log-output-with-rotating-file-handler\|JP-088]] | Compress Log Output with Rotating File Handler | candidate | — | — |
| [[JP-089-only-process-closest-obstacle-per-angular-sector\|JP-089]] | Only Process Closest Obstacle Per Angular Sector for CBF | candidate | — | — |
| [[JP-090-use-array-array-instead-of-python-list-for-obsta\|JP-090]] | Use `array.array` Instead of Python List for Obstacle Buffer | candidate | — | — |
| [[JP-091-precompute-static-gui-element-references\|JP-091]] | Precompute Static GUI Element References | candidate | — | — |
| [[JP-092-fused-multiply-add-for-mecanum-kinematics\|JP-092]] | Fused Multiply-Add for Mecanum Kinematics | candidate | — | — |
| [[JP-093-publish-cbf-filtered-velocity-as-a-ros-topic-for\|JP-093]] | Publish CBF-Filtered Velocity as a ROS Topic for Debugging | candidate | — | — |
| [[JP-094-use-lru-cache-for-repeated-euler-angle-conversio\|JP-094]] | Use `lru_cache` for Repeated Euler Angle Conversions | candidate | — | — |
| [[JP-095-reduce-yolo-input-resolution-dynamically-based-o\|JP-095]] | Reduce YOLO Input Resolution Dynamically Based on Speed | candidate | — | — |
| [[JP-096-use-websocket-bufferedamount-to-back-pressure-co\|JP-096]] | Use `WebSocket.bufferedAmount` to Back-Pressure Commands | candidate | — | — |
| [[JP-097-export-yolo-model-to-tensorrt-for-jetson-orin-ha\|JP-097]] | Export YOLO Model to TensorRT for Jetson Orin Hardware Acceleration | candidate | — | — |
| [[JP-098-add-http-response-gzip-compression-and-cache-con\|JP-098]] | Add HTTP Response Gzip Compression and Cache-Control Headers for GUI Assets | candidate | — | — |
| [[JP-099-downsample-lidar-laserscan-points-before-safety\|JP-099]] | Downsample Lidar LaserScan Points Before Safety Evaluation | candidate | — | — |
| [[JP-100-implement-a-simple-linear-kalman-filter-for-velo\|JP-100]] | Implement a Simple Linear Kalman Filter for Velocity Estimation Fallback | candidate | — | — |
| [[JP-101-consolidate-telemetry-and-control-websocket-mess\|JP-101]] | Consolidate Telemetry and Control Websocket Messages Into a Single Event Loop | candidate | — | — |
| [[JP-102-compress-camera-video-streams-using-webp-or-lowe\|JP-102]] | Compress Camera Video Streams Using WebP or Lower Quality JPEGs | candidate | — | — |
| [[JP-103-use-asynchronous-serial-i-o-for-ros-board-commun\|JP-103]] | Use Asynchronous Serial I/O for ROS Board Communication | candidate | — | — |
| [[JP-104-pre-compile-or-cache-mecanum-jacobian-matrices\|JP-104]] | Pre-Compile or Cache Mecanum Jacobian Matrices | candidate | — | — |
| [[JP-105-render-lidar-map-visualization-on-the-frontend-u\|JP-105]] | Render Lidar Map Visualization on the Frontend Using Offscreen Canvas or WebGL | candidate | — | — |
| [[JP-106-apply-adaptive-sleep-frequencies-to-the-motion-l\|JP-106]] | Apply Adaptive Sleep Frequencies to the Motion Loop | candidate | — | — |
| [[JP-107-apply-half-precision-float16-quantization-to-the\|JP-107]] | Apply Half-Precision (Float16) Quantization to the Velocity Predictor MLP | candidate | — | — |
| [[JP-108-avoid-string-concatenation-and-formatting-in-log\|JP-108]] | Avoid String Concatenation and Formatting in Logger Hot-Paths | candidate | — | — |
| [[JP-109-enable-tcp-nodelay-on-websockets-for-ultra-low-c\|JP-109]] | Enable TCP_NODELAY on Websockets for Ultra-Low Control Latency | candidate | — | — |
| [[JP-110-execute-websocket-frame-sending-in-non-blocking\|JP-110]] | Execute WebSocket Frame Sending in Non-Blocking Tasks | candidate | — | — |
| [[JP-111-restrict-lidar-laserscan-cbf-constraints-to-dire\|JP-111]] | Restrict Lidar LaserScan CBF Constraints to Direction of Travel | candidate | — | — |
| [[JP-112-precompute-and-cache-static-trigonometry-values\|JP-112]] | precompute and Cache Static Trigonometry Values in Velocity Estimation | candidate | — | — |
| [[JP-113-throttle-gamepad-joystick-event-broadcast-rate-t\|JP-113]] | Throttle Gamepad/Joystick Event Broadcast Rate to 50 Hz | candidate | — | — |
| [[JP-114-pre-allocate-input-arrays-for-yolo-inference-pre\|JP-114]] | Pre-Allocate Input Arrays for YOLO Inference Preprocessing | candidate | — | — |
| [[JP-115-implement-neural-network-weight-pruning-on-the-v\|JP-115]] | Implement Neural Network Weight Pruning on the Velocity Estimator MLP | candidate | — | — |
| [[JP-116-utilize-fast-binary-serialization-e-g-messagepac\|JP-116]] | Utilize Fast binary serialization (e.g. MessagePack) for WebSockets | candidate | — | — |
| [[JP-117-use-torch-inference-mode-instead-of-torch-no-gra\|JP-117]] | Use `torch.inference_mode()` Instead of `torch.no_grad()` for Velocity Estimator | candidate | — | — |
| [[JP-118-offload-heavy-callback-processing-in-ros2bridge\|JP-118]] | Offload Heavy Callback Processing in ROS2Bridge to an Executor Queue | candidate | — | — |
| [[JP-119-cache-astracamera-configurations-and-properties\|JP-119]] | Cache AstraCamera Configurations and Properties | candidate | — | — |
| [[JP-120-batch-dom-writes-using-requestanimationframe\|JP-120]] | Batch DOM Writes Using requestAnimationFrame | candidate | — | — |

## July — Improvements (`J-`)

28 ideas, 0 implemented — source: `July_improvement_ideas.md`

| ID | Title | Status | ROI | Domain |
| :-- | :-- | :-- | :-- | :-- |
| [[J-01-zero-copy-pre-allocated-numpy-view-for-mlp-featu\|J-01]] | Zero-Copy Pre-Allocated NumPy View for MLP Feature Normalization | candidate | High | Performance |
| [[J-02-motion-projected-centroid-association-for-cross\|J-02]] | Motion-Projected Centroid Association for Cross-Path Tracking | candidate | High | Architecture |
| [[J-03-onnx-runtime-fp16-arm-neon-vectorized-model-exec\|J-03]] | ONNX Runtime FP16 / ARM NEON Vectorized Model Execution | candidate | Medium-High | Performance |
| [[J-04-two-stage-decoupled-vision-and-mlp-inference-pip\|J-04]] | Two-Stage Decoupled Vision and MLP Inference Pipeline | candidate | High | Sensor Fusion |
| [[J-05-closest-point-of-approach-cpa-gating-for-ttc-spe\|J-05]] | Closest Point of Approach (CPA) Gating for TTC Speed Scaling | candidate | High | Architecture |
| [[J-06-vectorized-circular-numpy-buffer-for-sliding-win\|J-06]] | Vectorized Circular NumPy Buffer for Sliding Window Feature Extraction | candidate | High | Performance |
| [[J-07-intrinsic-camera-calibration-distortion-correcti\|J-07]] | Intrinsic Camera Calibration & Distortion Correction for Centroids | candidate | High | Sensor Fusion |
| [[J-08-tracking-confidence-gating-for-navigation-speed\|J-08]] | Tracking Confidence Gating for Navigation Speed Scaling & Bypass Decisions | candidate | High | Architecture |
| [[J-09-direct-hardware-level-spatial-detection-intake-t\|J-09]] | Direct Hardware-Level Spatial Detection Intake to Bypass Host Vision CPU Overhead | candidate | High | Sensor Fusion |
| [[J-10-backward-linear-extrapolation-padding-for-instan\|J-10]] | Backward Linear Extrapolation Padding for Instantaneous Track Initialization | candidate | High | Architecture |
| [[J-11-euclidean-norm-acceleration-clamping-with-true-t\|J-11]] | Euclidean Norm Acceleration Clamping with True Time-Delta Scaling | candidate | High | Architecture |
| [[J-12-true-zero-copy-pagelocked-pinned-host-buffers-fo\|J-12]] | True Zero-Copy Pagelocked (Pinned) Host Buffers for Async TensorRT DMA | candidate | High | Performance |
| [[J-13-dynamic-predictive-control-barrier-function-dp-c\|J-13]] | Dynamic Predictive Control Barrier Function (DP-CBF) with Obstacle Velocity Drift | candidate | High | Architecture |
| [[J-14-global-coordinate-frame-velocity-export-for-rota\|J-14]] | Global Coordinate Frame Velocity Export for Rotational-Invariant Prediction | candidate | High | Architecture |
| [[J-15-virtual-pointcloud-forward-projection-topic-for\|J-15]] | Virtual PointCloud Forward-Projection Topic for Zero-Plugin Nav2 Costmap Integration | candidate | High | Architecture |
| [[J-16-asymmetric-kinematic-rate-limiter-for-speed-scal\|J-16]] | Asymmetric Kinematic Rate Limiter for Speed Scale Transitions | candidate | High | Performance |
| [[J-17-nav2-action-feedback-telemetry-harvesting-for-dy\|J-17]] | Nav2 Action Feedback Telemetry Harvesting for Dynamic Replanning Triggers | candidate | High | Architecture |
| [[J-18-ring-buffered-full-horizon-lidar-sector-accumula\|J-18]] | Ring-Buffered Full-Horizon LiDAR Sector Accumulation for Sub-Scan Latency Reduction | candidate | High | Sensor Fusion |
| [[J-19-dynamic-approach-gated-mlp-horizon-to-capture-hi\|J-19]] | Dynamic Approach-Gated MLP Horizon to Capture High-Speed Hazards Beyond 1.8m | candidate | High | Architecture |
| [[J-20-modal-cluster-depth-extraction-via-1d-histogram\|J-20]] | Modal Cluster Depth Extraction via 1D Histogram Density Peak Matching for Occlusion-Robust 3D Centroids | candidate | High | Sensor Fusion |
| [[J-21-imu-odometry-residual-fusion-for-real-time-mecan\|J-21]] | IMU-Odometry Residual Fusion for Real-Time Mecanum Lateral Slip Compensation | candidate | High | Sensor Fusion |
| [[J-22-decoupled-kinematic-acceleration-clamping-from-c\|J-22]] | Decoupled Kinematic Acceleration Clamping from Confidence Gating | candidate | High | Architecture |
| [[J-23-direct-gpu-cuda-preprocessing-kernel-for-yolov8\|J-23]] | Direct GPU CUDA Preprocessing Kernel for YOLOv8/v10 Input Normalization | candidate | High | Performance |
| [[J-24-lock-free-triple-buffering-swap-chain-for-zero-c\|J-24]] | Lock-Free Triple-Buffering Swap Chain for Zero-Copy Depth Frame IPC | candidate | High | Performance |
| [[J-25-ego-motion-invariant-feature-extraction-via-inst\|J-25]] | Ego-Motion Invariant Feature Extraction via Instantaneous Body-Frame Back-Projection | candidate | High | Architecture |
| [[J-26-true-distance-normalized-pure-pursuit-curvature\|J-26]] | True Distance-Normalized Pure Pursuit Curvature with MLP Interception Lookahead | candidate | High | Architecture |
| [[J-27-hardware-subpixel-disparity-enablement-for-mediu\|J-27]] | Hardware Subpixel Disparity Enablement for Medium/Long-Range Quantization Velocity Spikes | candidate | High | Sensor Fusion |
| [[J-28-ego-velocity-compensation-in-travel-aligned-rela\|J-28]] | Ego-Velocity Compensation in Travel-Aligned Relative Velocity Vectors for True Predictive TTC Braking | candidate | High | Architecture |

## August — Improvements (`A-`)

58 ideas, 0 implemented — source: `august_improvement_ideas.md`

| ID | Title | Status | ROI | Domain |
| :-- | :-- | :-- | :-- | :-- |
| [[A-01-global-optimal-bipartite-matching-for-cross-path\|A-01]] | Global Optimal Bipartite Matching for Cross-Path Tracking | candidate | High | Architecture |
| [[A-02-connected-components-for-zero-overhead-depth-blo\|A-02]] | Connected Components for Zero-Overhead Depth Blob Extraction | candidate | High | Performance |
| [[A-03-chunked-serial-payload-reading-for-reduced-sysca\|A-03]] | Chunked Serial Payload Reading for Reduced Syscall Overhead | candidate | High | Performance |
| [[A-04-vectorized-struct-unpacking-for-telemetry-parsin\|A-04]] | Vectorized Struct Unpacking for Telemetry Parsing | candidate | High | Performance |
| [[A-05-thread-safe-serial-writes-to-prevent-packet-corr\|A-05]] | Thread-Safe Serial Writes to Prevent Packet Corruption | candidate | High | Architecture |
| [[A-06-graceful-thread-termination-and-non-blocking-i-o\|A-06]] | Graceful Thread Termination and Non-Blocking I/O | candidate | High | Code Quality |
| [[A-07-eliminate-o-n-2-list-insertions-in-window-paddin\|A-07]] | Eliminate O(N^2) List Insertions in Window Padding | candidate | High | Performance |
| [[A-08-refactor-duplicated-uart-transmission-logic-into\|A-08]] | Refactor Duplicated UART Transmission Logic into a Helper Method | candidate | High | Code Quality |
| [[A-09-parameterize-hardcoded-camera-intrinsics-and-con\|A-09]] | Parameterize Hardcoded Camera Intrinsics and Constants | candidate | Medium | Architecture |
| [[A-10-int8-fp16-quantization-for-the-velocity-mlp-mode\|A-10]] | INT8/FP16 Quantization for the Velocity MLP Model | candidate | High | Performance |
| [[A-11-migrate-mlp-inference-to-onnx-runtime-tensorrt\|A-11]] | Migrate MLP Inference to ONNX Runtime / TensorRT | candidate | High | Performance |
| [[A-12-decouple-opencv-visualization-from-the-inference\|A-12]] | Decouple OpenCV Visualization from the Inference Loop | candidate | High | Architecture |
| [[A-13-eliminate-bare-except-clauses-and-implement-robu\|A-13]] | Eliminate Bare Except Clauses and Implement Robust UART Error Handling | candidate | High | Code Quality |
| [[A-14-use-event-driven-synchronization-for-asynchronou\|A-14]] | Use Event-Driven Synchronization for Asynchronous UART Reads | candidate | High | Architecture |
| [[A-15-remove-redundant-runtime-jit-tracing\|A-15]] | Remove Redundant Runtime JIT Tracing | candidate | High | Code Quality |
| [[A-16-embed-scaling-operations-directly-into-the-model\|A-16]] | Embed Scaling Operations Directly into the Model Graph | candidate | High | Architecture |
| [[A-17-optimize-opencv-morphological-operations-with-se\|A-17]] | Optimize OpenCV Morphological Operations with Separable/Smaller Kernels | candidate | Medium | Performance |
| [[A-18-vectorize-sliding-window-features-using-numpy-in\|A-18]] | Vectorize Sliding Window Features using NumPy Instead of Python Loops | candidate | High | Performance |
| [[A-19-optimize-checksum-validation-using-native-sum-in\|A-19]] | Optimize Checksum Validation using Native sum() in UART parsing | candidate | High | Performance |
| [[A-20-avoid-pulling-colorized-depth-frame-if-visualiza\|A-20]] | Avoid Pulling Colorized depth_frame if Visualization is Disabled | candidate | Medium | Performance |
| [[A-21-prioritize-tracking-proximal-obstacles-when-at-t\|A-21]] | Prioritize Tracking Proximal Obstacles When at Tracking Capacity | candidate | High | Architecture |
| [[A-22-use-command-queue-instead-of-blind-time-sleep-po\|A-22]] | Use Command Queue Instead of Blind time.sleep Post-Write Delays | candidate | High | Performance |
| [[A-23-eliminate-redundant-cv2-boundingrect-computation\|A-23]] | Eliminate Redundant cv2.boundingRect Computations in Blob Extraction | candidate | Medium | Code Quality |
| [[A-24-extrapolate-state-for-coasting-tracks-instead-of\|A-24]] | Extrapolate State for Coasting Tracks instead of Re-evaluating Stale History | candidate | High | Architecture |
| [[A-25-vectorize-kinematic-stop-trigger-gating-in-featu\|A-25]] | Vectorize Kinematic Stop-Trigger Gating in Feature Extraction | candidate | Medium | Performance |
| [[A-26-depth-ema-applied-after-global-projection-makes\|A-26]] | Depth EMA Applied After Global Projection Makes `alpha_z` a No-Op on MLP Features | candidate | High | Architecture |
| [[A-27-proximity-ranked-centroid-truncation-before-the\|A-27]] | Proximity-Ranked Centroid Truncation Before the `MAX_OBSTACLES` Cutoff | candidate | High | Architecture |
| [[A-28-single-pass-downsampled-cv2-inrange-mask-shared\|A-28]] | Single-Pass Downsampled `cv2.inRange` Mask Shared by the Empty-Frame Fast Path | candidate | High | Performance |
| [[A-29-net-displacement-stop-gate-to-restore-kinematic\|A-29]] | Net-Displacement Stop Gate to Restore Kinematic Stop Detection Above the Depth Noise Floor | candidate | High | Architecture |
| [[A-30-translation-normalized-serving-features-contradi\|A-30]] | Translation-Normalized Serving Features Contradict the Absolute-Coordinate Scaler the Model Was Fit On | candidate | High | Architecture |
| [[A-31-blob-depth-reference-sampled-from-a-single-bound\|A-31]] | Blob Depth Reference Sampled From a Single Bounding-Box-Centre Pixel Inverts the Range-Adaptive Area Gate | candidate | High | Architecture |
| [[A-32-gate-emitted-zero-velocities-poison-prev-estimat\|A-32]] | Gate-Emitted Zero Velocities Poison `_prev_estimates` and Impose a 0.5 s Ramp on Every Gate Release | candidate | High | Architecture |
| [[A-33-unconditional-depth-colourisation-in-inference-l\|A-33]] | Unconditional Depth Colourisation in `_inference_loop` for a Debug Overlay That Is Off in Production | candidate | High | Performance |
| [[A-34-visual-gating-reprojects-oak-d-centroids-with-as\|A-34]] | Visual Gating Reprojects OAK-D Centroids With Astra Intrinsics and No Inter-Camera Extrinsic | candidate | High | Sensor Fusion |
| [[A-35-stale-odometry-fallback-substitutes-the-map-orig\|A-35]] | Stale-Odometry Fallback Substitutes the Map Origin and Annihilates Every Track | candidate | High | Architecture |
| [[A-36-confidence-scaled-velocity-export-compounds-the\|A-36]] | Confidence-Scaled Velocity Export Compounds the Padding Under-Report and Inverts the Braking Response | candidate | High | Architecture |
| [[A-37-variable-batch-size-defeats-the-traced-graph-s-s\|A-37]] | Variable Batch Size Defeats the Traced Graph's Shape Specialisation on Every Cycle | candidate | High | Performance |
| [[A-38-no-ground-plane-rejection-the-floor-is-inside-th\|A-38]] | No Ground-Plane Rejection — the Floor Is Inside the Acceptance Band and Merges With Every Pedestrian | candidate | High | Sensor Fusion |
| [[A-39-zero-phase-savitzky-golay-window-smoothing-befor\|A-39]] | Zero-Phase Savitzky–Golay Window Smoothing Before Differencing | candidate | High | Architecture |
| [[A-40-empty-frame-fast-path-skips-tracker-aging-and-pr\|A-40]] | Empty-Frame Fast Path Skips Tracker Aging and Preserves a Stale Window Across a Depth Blackout | candidate | High | Architecture |
| [[A-41-strided-2-2-views-force-hidden-opencv-contiguity\|A-41]] | Strided `[::2, ::2]` Views Force Hidden OpenCV Contiguity Copies and 2× Cache-Line Overfetch | candidate | Medium-High | Performance |
| [[A-42-depth-intrinsics-read-from-cam-a-while-the-map-i\|A-42]] | Depth Intrinsics Read From CAM_A While the Map Is Aligned to CAM_B in the Non-Spatial Pipeline | candidate | High | Sensor Fusion |
| [[A-43-depth-frames-are-consumed-on-a-free-running-host\|A-43]] | Depth Frames Are Consumed on a Free-Running Host Clock With No Capture Timestamp — the Feature Δt Is Assumed, Never Measured | candidate | High | Sensor Fusion |
| [[A-44-full-frame-uint16-float32-depth-conversion-at-80\|A-44]] | Full-Frame `uint16`→`float32` Depth Conversion at 80 fps Capture for a 10 Hz Consumer | candidate | High | Performance |
| [[A-45-frame-0-displacement-hardcoded-to-zero-while-the\|A-45]] | Frame-0 Displacement Hardcoded to Zero While the Scaler Shows `dx₀` Was a Real Displacement in Training | candidate | High | Architecture |
| [[A-46-acceleration-limiter-differences-two-velocities\|A-46]] | Acceleration Limiter Differences Two Velocities Expressed in Different Rotating Body Frames | candidate | High | Architecture |
| [[A-47-exported-track-position-is-the-stale-body-frame\|A-47]] | Exported Track Position Is the Stale Body-Frame Centroid From the Last Matched Frame While the Re-Referenced One Sits Unused | candidate | High | Architecture |
| [[A-48-scaler-y-vs-scaler-x-audit-79-of-the-dx-channel\|A-48]] | `scaler_y` vs `scaler_X` Audit — 79% of the `dx` Channel Is Noise, and Nothing Cross-Checks the MLP Against the Free Finite-Difference Estimate | candidate | High | Architecture |
| [[A-49-pedestrian-topics-republished-at-2-the-producer\|A-49]] | Pedestrian Topics Republished at 2× the Producer Rate and Never Published Empty — Unbounded Stale Markers and a `/pedestrian_states` Array That Never Clears | candidate | High | Performance |
| [[A-50-stereodepth-built-with-zero-on-device-post-proce\|A-50]] | StereoDepth Built With Zero On-Device Post-Processing — Every Depth Cleanup Pass Is Paid on the Jetson at 10 Hz | candidate | High | Sensor Fusion |
| [[A-51-blob-depth-median-decimated-by-a-size-dependent\|A-51]] | Blob Depth Median Decimated by a Size-Dependent Stride — a Discontinuous Subsample That Steps `Z` and Buys ~10 µs | candidate | High | Architecture |
| [[A-52-traced-graph-is-never-frozen-batchnorm-and-dropo\|A-52]] | Traced Graph Is Never Frozen — BatchNorm and Dropout Dispatched as Live Ops in a 40→256→128→64→2 MLP | candidate | High | Performance |
| [[A-53-the-safety-critical-speed-scaler-reads-velocitie\|A-53]] | The Safety-Critical Speed Scaler Reads Velocities Through a WebSocket JSON Round-Trip, Not the ROS Topic Published Beside It | candidate | High | Performance |
| [[A-54-a-0-185-m-mount-with-a-24-8-vertical-half-fov-cr\|A-54]] | A 0.185 m Mount With a 24.8° Vertical Half-FOV Crops the Pedestrian at the Waist — the Depth Reference Migrates to the Legs and Picks Up Gait | candidate | High | Sensor Fusion |
| [[A-55-the-feature-window-round-trips-through-python-ob\|A-55]] | The Feature Window Round-Trips Through Python Objects Twice Between Two NumPy Arrays — 90 Scalar `np.clip` Dispatches per Cycle to Fill a Pre-Allocated Tensor | candidate | High | Performance |
| [[A-56-self-lidar-is-assigned-and-never-read-a-2-cm-pla\|A-56]] | `self.lidar` Is Assigned and Never Read — a 2 cm Planar Ranger Is Wired In While the Radial Channel Runs on 12 cm Stereo Quantisation Steps | candidate | High | Sensor Fusion |
| [[A-57-the-depth-frame-is-from-t-and-the-pose-is-from-t\|A-57]] | The Depth Frame Is From `t−τ` and the Pose Is From `t` — the `twist` the Docstring Reserves for Ego-Motion Compensation Is Fetched Every Cycle and Never Read | candidate | High | Architecture |
| [[A-58-one-scaler-params-json-behind-a-module-constant\|A-58]] | One `scaler_params.json` Behind a Module Constant Serves Two Different Model Artifacts, and the Runtime Model Switch Cannot Swap It | candidate | High | Architecture |

