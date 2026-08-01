# August 2026 Improvement Ideas ROI Analysis

This document records the ROI rankings and evaluation metrics for the enhancement ideas logged in `august_improvement_ideas.md` for the predictive planning and local velocity estimation project.

## Last Updated
2026-08-01: Added 37 ideas (A-01–A-37) across tiers. 36 new High-ROI, 1 new Medium-ROI entries added.

---

## 1. Already Implemented Ideas
The following **46 ideas** have already been completed and integrated. Each was verified against the inline `(Idea NN)` markers present in `src/` as of this consolidation.

Carried forward from `ideas/June_roi_analysis.md` §1 and confirmed still present:
*   **Idea 1**: Ego-Motion Compensation in `VelocityEstimator`
*   **Idea 2**: Live ROS2 Topic Publishing for Nav2/MPPI Integration
*   **Idea 9**: RViz2 Spatio-Temporal Trajectory & Velocity Vector Markers
*   **Idea 11**: Encoders-IMU Slip Compensation using EKF Velocity Feedback
*   **Idea 19**: Depth-Aware 3D Centroid Reconstruction for Velocity Scaling
*   **Idea 21**: Direct Meters-Based Depth Processing to Avoid Normalization Artifacts
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
*   **Idea 72**: Pure NumPy Feature Normalization (Eliminating Scikit-Learn)
*   **Idea 141**: Active Lateral Centering Potential Field for Corridor Navigation
*   **Idea 143**: Dynamic Point Cloud Downsampling with Adaptive Voxel Grid Gating
*   **Idea 145**: Zero-Copy Memory-Mapped Frame Allocation via Shared Memory IPC
*   **Idea 146**: Visual-LiDAR Geometric Depth Fusion (Sensor Fusion Gating)

Integrated since the June analysis (new `(Idea NN)` markers in `src/`):
*   **Idea 80**: Rear-Sector LiDAR Protection for Backing Recovery
*   **Idea 81**: Immediate Depth Downsampling for Centroid Extraction
*   **Idea 87**: High-Performance orjson Serialization
*   **Idea 91**: Vectorized Pairwise Distance Matrix Broadcasting
*   **Idea 94**: Spatio-Temporal Static Saliency Masking for Doorways
*   **Idea 106**: Downscaled Depth Visualization Encoding
*   **Idea 108**: Kinematic Stop-Trigger Gating
*   **Idea 109**: Centroid Presence Verification Filter (Track Initiation Gate)
*   **Idea 110**: Dynamic Corridor Width Clearance Scaling
*   **Idea 111**: Serial Command Write Rate Optimization (30Hz Throttle)
*   **Idea 116**: Pre-Allocated PyTorch Input Tensor Reuse
*   **Idea 117**: Non-Blocking WebSockets Payload Compression
*   **Idea 121**: Vectorized Centroid Global Transformation via Matrix Dot Products
*   **Idea 122**: Dynamic Path Deceleration via Predictive TTC Hysteresis
*   **Idea 136**: Vectorized Frame Downsampling via Slice Offsets
*   **Idea 137**: Pre-Compiled PyTorch Modules via JIT Tracing (Trace-Optimization)
*   **Idea 152**: Physically Constrained MLP Output Gating
*   **Idea 197**: EMA-Smoothed Wall Clearance Values for APF Jitter Prevention
*   **Idea 200**: Non-Blocking Recovery Backing via Timer-Based State
*   **Idea 203**: Multi-Frame Confirmation Counter for `_front_is_continuous_wall`
*   **Idea 209**: Cache `start_yaw` Trigonometry Constants at Segment Transitions
*   **Idea 224**: Forward Speed Ramp-Up Delay After Pause Release
*   **Idea 225**: EKF Pose Staleness Guard in `get_robot_pose_and_twist`
*   **Idea 230**: Depth Frame Staleness Gate in `_inference_loop`

**Verification notes.** Three carried-forward entries warrant a caveat, and ten were dropped:
*   **Idea 56** survives only in `src/drivers_x3.py`; the velocity pipeline uses raw boolean NumPy comparisons instead (`src/velocity_estimator.py:224, 235`), which is what Idea A-07 proposes to correct.
*   **Idea 146** is present but neutered — the discard branch is commented out and replaced by `pass` at `src/velocity_estimator.py:312–313`. Idea A-13 covers restoring it.
*   **Idea 52** is present and is now itself the defect Idea A-30 proposes to delete.
*   Dropped from the June list — no `(Idea NN)` marker and no corresponding implementation found in `src/`: Ideas 3, 13, 17, 18, 22, 26, 29, 30, 71 and 144. Idea 13's TTC speed scaling now lives in the browser client rather than in Python, which is the arrangement Idea A-32 flags as a latency defect.

---

## 2. High-ROI Tier (Rank 1 - 36)
*These enhancements require minimal development effort (a deleted line, a reordered statement, a value already computed and discarded, or a standard formula) but deliver substantial improvements in CPU/RAM usage, execution efficiency, stability, or model accuracy.*

### 1. Idea A-09: Translation-Normalized Serving Features Contradict the Absolute-Coordinate Scaler the Model Was Fit On
*   **Investment:** Extremely Low. Delete two subtractions at `src/velocity_estimator.py:402–403`.
*   **Return:** Extremely High. Twenty of the forty input channels currently collapse to a frozen constant after normalisation; restoring parity with the shipped artifact returns half the model's input to being real data. Nothing else on this list unlocks that much of the network for that little work.

### 2. Idea A-26: Exported Track Position Is the Stale Body-Frame Centroid From the Last Matched Frame While the Re-Referenced One Sits Unused
*   **Investment:** Extremely Low. Read the correct value — lines 529–533 already compute it and throw it away.
*   **Return:** Very High. Removes a stale-frame position error that scales with target speed and fires precisely during turns, on the number the CBF, the proximity gate and the speed scaler all treat as ground truth.

### 3. Idea A-30: Blob Depth Median Decimated by a Size-Dependent Stride
*   **Investment:** Extremely Low. Delete two lines (`src/velocity_estimator.py:277–279`).
*   **Return:** High. Removes a recurring 1–3 cm discontinuous step in the depth reference — the single number that becomes every `dx` the MLP reads — in exchange for a saving under 0.01% of the cycle budget.

### 4. Idea A-15: Confidence-Scaled Velocity Export Compounds the Padding Under-Report and Inverts the Braking Response
*   **Investment:** Extremely Low. Stop multiplying the exported velocity by a visibility ratio.
*   **Return:** Very High. Safety-critical sign correction: the robot currently brakes *least* for the tracks it has seen *least*, which is exactly backwards.

### 5. Idea A-11: Gate-Emitted Zero Velocities Poison `_prev_estimates` and Impose a 0.5 s Ramp on Every Gate Release
*   **Investment:** Extremely Low. Exclude gate-emitted zeros from the acceleration-limiter memory.
*   **Return:** Very High. Eliminates a 0.5 s ramp on every gate release — half a second of under-reported speed each time a pedestrian re-enters the acceptance band.

### 6. Idea A-12: Unconditional Depth Colourisation in `_inference_loop` for a Debug Overlay That Is Off in Production
*   **Investment:** Extremely Low. Guard one call behind the `DISPLAY` check that already exists at line 618.
*   **Return:** High. Removes a full-frame colormap conversion at 10 Hz that no production consumer ever reads.

### 7. Idea A-37: One `scaler_params.json` Behind a Module Constant Serves Two Different Model Artifacts
*   **Investment:** Very Low. One path derivation, one digest field, and moving three assignments below the `try`.
*   **Return:** Very High. Closes a silent, unbounded gain error on the exported velocity — a 30% narrower fine-tune distribution alone yields a 43% over-report on $v_x$ with no visible symptom — and turns an invisible half-swap on a failed model switch into an operator-visible failure.

### 8. Idea A-31: Traced Graph Is Never Frozen — BatchNorm and Dropout Dispatched as Live Ops
*   **Investment:** Extremely Low. One call: `torch.jit.freeze` after the existing trace at line 187.
*   **Return:** High. Folds inference-time-constant BatchNorm and Dropout out of a 40→256→128→64→2 graph, cutting per-call dispatch on the hot path with zero behavioural change.

### 9. Idea A-19: Empty-Frame Fast Path Skips Tracker Aging and Preserves a Stale Window Across a Depth Blackout
*   **Investment:** Very Low. Call `self._tracker.update([], [])` on the fast path and clear `_prev_estimates`.
*   **Return:** Very High. Today a blackout leaves five tracks alive indefinitely at `conf = 1.0`, then splices a saturated 2.5 m/s phantom across the gap that persists for a full second.

### 10. Idea A-25: Acceleration Limiter Differences Two Velocities Expressed in Different Rotating Body Frames
*   **Investment:** Very Low. Rotate the previous velocity into the current frame before differencing.
*   **Return:** High. Removes a limiter artefact that both shrinks *and* rotates the exported vector during turns — the exact manoeuvre in which the avoidance decision is made.

### 11. Idea A-14: Stale-Odometry Fallback Substitutes the Map Origin and Annihilates Every Track
*   **Investment:** Very Low. Cache the last good pose and reuse it instead of substituting the identity transform.
*   **Return:** Very High. A single dropped-odom window currently costs up to 2 s of total blindness plus a saturated phantom velocity, on a robot whose whole safety case rests on this estimator.

### 12. Idea A-36: The Depth Frame Is From `t−τ` and the Pose Is From `t`
*   **Investment:** Very Low. Two lines back-dating the pose with the `twist` the callback already returns and the estimator already receives.
*   **Return:** Very High. Removes ≈0.05 m injected into a single `dy` sample at every turn onset — 68% of that channel's entire training spread — plus a sustained 0.10 m/s phantom through steady turns, with the wrong sign for avoidance.

### 13. Idea A-24: Frame-0 Displacement Hardcoded to Zero While the Scaler Shows `dx₀` Was a Real Displacement in Training
*   **Investment:** Very Low. Widen one deque by a single slot and difference against the extra sample.
*   **Return:** High. Restores one of ten window frames to carrying real displacement rather than a structural zero the scaler proves was never zero in training. Cost is 24 bytes per track.

### 14. Idea A-23: Full-Frame `uint16`→`float32` Depth Conversion at 80 fps Capture for a 10 Hz Consumer
*   **Investment:** Very Low. Defer the conversion to the consumer, or keep the native dtype through the range gate.
*   **Return:** Very High. Removes a 3.6 MB allocate-and-convert on the capture thread at up to 8× the rate anything reads it — the largest single unnecessary memory-bandwidth item in the pipeline.

### 15. Idea A-10: Blob Depth Reference Sampled From a Single Bounding-Box-Centre Pixel Inverts the Range-Adaptive Area Gate
*   **Investment:** Very Low. Take the reference from the blob's own depth population instead of one pixel.
*   **Return:** High. One noisy pixel currently drives an area gate that scales as $(1.5/z)^2$, so a single bad sample can swing the admission threshold by an order of magnitude.

### 16. Idea A-06: Proximity-Ranked Centroid Truncation Before the `MAX_OBSTACLES` Cutoff
*   **Investment:** Very Low. Sort by range before the two truncation points at lines 319/361 and 103–105.
*   **Return:** High. Stops a pedestrian at 1.2 m being discarded in favour of four wall fragments at 3.5–4.0 m selected by raster order.

### 17. Idea A-16: Variable Batch Size Defeats the Traced Graph's Shape Specialisation on Every Cycle
*   **Investment:** Very Low. Pad the batch to the traced shape, or trace for the shape actually served.
*   **Return:** High. The batch dimension oscillates every cycle and is essentially never the 5 the trace was specialised on, so the specialisation Idea 137 paid for is discarded each frame.

### 18. Idea A-28: Pedestrian Topics Republished at 2× the Producer Rate and Never Published Empty
*   **Investment:** Very Low. Publish at the producer rate and publish the empty array when tracks vanish.
*   **Return:** High. Halves ≈2400 object constructions per second on the asyncio event loop and stops `/pedestrian_states` and the marker array retaining pedestrians that no longer exist.

### 19. Idea A-07: Single-Pass Downsampled `cv2.inRange` Mask Shared by the Empty-Frame Fast Path
*   **Investment:** Very Low. One `cv2.inRange` on the downsampled frame, reused by both the fast path and the extractor.
*   **Return:** High. Replaces multiple full-resolution boolean temporaries and passes with one SIMD call, on the hottest loop in the file.

### 20. Idea A-05: Depth EMA Applied After Global Projection Makes `alpha_z` a No-Op on MLP Features
*   **Investment:** Very Low. Apply the filter where the features are built rather than downstream of the projection.
*   **Return:** High. The depth smoother Idea 43 added currently has no effect whatsoever on anything the model reads — turning it back on is close to free.

### 21. Idea A-08: Net-Displacement Stop Gate to Restore Kinematic Stop Detection Above the Depth Noise Floor
*   **Investment:** Low. Replace the per-frame threshold with a net-displacement test over the window.
*   **Return:** High. The current 0.01 m per-frame threshold sits below the sensor noise floor, so the stop gate fires essentially at random rather than on stopped pedestrians.

### 22. Idea A-29: StereoDepth Built With Zero On-Device Post-Processing
*   **Investment:** Low. Enable the on-device filters in `_build_pipeline`; no host code changes.
*   **Return:** Very High. Moves every depth cleanup pass off the Jetson CPU and onto silicon that is already idle, improving disparity *validity* — it stacks with Idea J-27's subpixel work, which improves disparity *resolution*.

### 23. Idea A-32: The Safety-Critical Speed Scaler Reads Velocities Through a WebSocket JSON Round-Trip
*   **Investment:** Low-Moderate. Move the consumer onto the typed `/pedestrian_states` topic already published beside it.
*   **Return:** Very High. Removes ≈55 ms mean and ≈108 ms worst-case transport delay from the braking decision — 0.08–0.15 m of pedestrian position error — and gives the typed topic its first subscriber.

### 24. Idea A-34: The Feature Window Round-Trips Through Python Objects Twice Between Two NumPy Arrays
*   **Investment:** Low-Moderate. Rewrite one 50-line function as batched array code over an $(N, T, 2)$ ring buffer.
*   **Return:** Very High. Cuts the feature stage ≈95% (0.30 ms → 15 µs per cycle; 900 scalar `np.clip` dispatches per second down to 10) and, more valuably, reduces Ideas A-18, A-22 and A-24 from cross-cutting edits to one-liners.

### 25. Idea A-27: `scaler_y` vs `scaler_X` Audit and a Finite-Difference Cross-Check
*   **Investment:** Low-Moderate. Offline analysis plus a cheap runtime residual comparison.
*   **Return:** Very High. Quantifies how much of the `dx` channel is noise rather than signal, and adds the only independent check on the MLP anywhere in the system — using an estimate the pipeline already computes for free.

### 26. Idea A-22: Depth Frames Consumed on a Free-Running Host Clock With No Capture Timestamp
*   **Investment:** Moderate. One timestamp field carried from the device through the driver to the estimator, plus a sequence check.
*   **Return:** Extremely High. Makes the 0.1 s interval the model was trained on an enforced property rather than an assumption; removes ≈0.12 m of structured, unsmoothable spread on `dx` at low depth rates and a ±12.5% always-on multiplicative error. It is a hard precondition for June's Idea 33 and for Idea A-18.

### 27. Idea A-21: Depth Intrinsics Read From CAM_A While the Map Is Aligned to CAM_B
*   **Investment:** Moderate. Read intrinsics from the socket the map is actually aligned to, and handle the runtime resolution flip.
*   **Return:** High. The depth map changes both frame and dimensions when the spatial pipeline falls back, and the back-projection constants track neither.

### 28. Idea A-13: Visual Gating Reprojects OAK-D Centroids With Astra Intrinsics and No Inter-Camera Extrinsic
*   **Investment:** Moderate. Add `get_intrinsics()`, gate against same-camera detections, and fail closed when unavailable.
*   **Return:** High. Turns Idea 146 from permanently disabled code into a working person filter — the cheapest available route to suppressing the static clutter that currently consumes the `MAX_OBSTACLES` budget.

### 29. Idea A-17: No Ground-Plane Rejection — the Floor Is Inside the Acceptance Band
*   **Investment:** Moderate. A height-based rejection pass before contour extraction.
*   **Return:** Very High. The floor is the largest connected component in the frame, clears the area gate by orders of magnitude, permanently consumes a track slot, and merges with every pedestrian standing on it.

### 30. Idea A-18: Zero-Phase Savitzky–Golay Window Smoothing Before Differencing
*   **Investment:** Moderate. One `savgol_filter` over the window axis — trivial once Idea A-34 lands, and unbiased only once Idea A-22 does.
*   **Return:** High. Attacks the ≈0.042 m per-frame sensor-noise term directly, without the phase lag a causal filter would add to a safety-critical estimate.

### 31. Idea A-35: `self.lidar` Is Assigned and Never Read
*   **Investment:** Moderate. A bearing-sorted linear scan and an inverse-variance fuse, behind a config flag and A/B'd against the camera-only path.
*   **Return:** Very High. Replaces a ±0.121 m stereo quantisation staircase at 2 m — 61% of the `dx` training spread, and the same size as a walking pedestrian's true per-frame displacement — with a ≈0.04 m measurement, and converts the dominant error from broadband to narrowband. Net of the ±0.07 m gait ripple the 0.11 m mount introduces.

### 32. Idea A-01: Global Optimal Bipartite Matching for Cross-Path Tracking
*   **Investment:** Moderate. Replace greedy nearest-centroid association with a Hungarian assignment over the cost matrix.
*   **Return:** High. Greedy matching in dict-insertion order swaps identities whenever two pedestrians cross, which resets `visible_count` and splices two people's trajectories into one window.

### 33. Idea A-02: Connected Components for Zero-Overhead Depth Blob Extraction
*   **Investment:** Moderate. Replace `findContours` plus per-contour `drawContours` masking with one `connectedComponentsWithStats` pass.
*   **Return:** High. Yields centroids, areas and bounding boxes in a single pass and removes the per-contour mask allocation entirely; supersedes several smaller extraction-stage items at once.

### 34. Idea A-03: Chunked Serial Payload Reading for Reduced Syscall Overhead
*   **Investment:** Low. Read the full frame in one call rather than byte-at-a-time.
*   **Return:** Moderate. Real CPU saving on the Rosmaster link, but off the velocity-estimation critical path — which is the only reason it ranks here rather than higher.

### 35. Idea A-04: Vectorized Struct Unpacking for Telemetry Parsing
*   **Investment:** Low. One `struct.unpack` over the whole payload instead of per-field slicing.
*   **Return:** Moderate. Same character as A-03: cheap and clean, but it does not touch the MLP pipeline.

### 36. Idea A-33: A 0.185 m Mount With a 24.8° Vertical Half-FOV Crops the Pedestrian at the Waist
*   **Investment:** High. Physical remount plus URDF and extrinsic updates — the only hardware item on this list.
*   **Return:** Very High. Fixes the root cause behind several software mitigations at once: the depth reference stops migrating to the legs and stops picking up gait. Ranked last in the tier purely on investment, not on payoff.

---

## 3. Medium-ROI Tier (Rank 37)
*This enhancement offers a solid execution-efficiency benefit, but it requires moderate profiling effort to confirm the win and its payoff is bounded by memory bandwidth rather than by a correctness defect.*

### 37. Idea A-20: Strided `[::2, ::2]` Views Force Hidden OpenCV Contiguity Copies and 2× Cache-Line Overfetch
*   **Investment:** Moderate. Replace the strided views with an explicit contiguous decimation so OpenCV is never handed a non-contiguous array.
*   **Return:** Medium-High. Removes hidden per-call copies and halves cache-line overfetch in the extraction stage, but the gain is bounded and overlaps with Idea A-02, which would restructure the same code path.

---

## 4. Low-ROI Tier
No ideas logged in August 2026 fell into this tier.
