# Project Improvement Ideas ROI Analysis

This document records the ROI rankings and evaluation metrics for the 137 enhancement ideas logged in `improvement_ideas.md` for the predictive planning and local velocity estimation project.

---

## 1. Already Implemented Ideas
The following **27 ideas** have already been completed and integrated:
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

---

## 2. High-ROI Tier (Rank 1 - 15)
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

---

## 3. Medium-ROI Tier (Rank 16 - 75)
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

---

## 4. Low-ROI Tier (Rank 76 - 110)
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
