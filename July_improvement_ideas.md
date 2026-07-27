# July 2026 Improvement Ideas & Enhancement Log

This document serves as the active improvement ideas log and architectural roadmap for the EE244 Computational Learning Project (Predictive Local Planning via Onboard Velocity Estimation) for **July 2026**.

> [!NOTE]
> For historical context and prior ideas generated during June 2026, refer to the centralized archive in the [`ideas/`](ideas) directory:
> - **Architectural Ideas (1–240)**: [ideas/June_architectural_ideas.md](ideas/June_architectural_ideas.md)
> - **Performance & Efficiency (1–120)**: [ideas/June_performance_ideas.md](ideas/June_performance_ideas.md)
> - **June ROI Analysis**: [ideas/June_roi_analysis.md](ideas/June_roi_analysis.md)

---

## 1. Prioritization & ROI Summary Matrix

Use this matrix to track active ideas and their ROI tier at a glance.

| Idea ID | Date Logged | Domain | Title | ROI Tier | Status |
| :---: | :---: | :--- | :--- | :---: | :---: |
| *e.g., J-01* | *2026-07-26* | *Architecture* | *Template entry: OAK-D Lite Domain Gap Self-Training* | *High* | *Planned* |
| **J-01** | 2026-07-26 | Performance | Zero-Copy Pre-Allocated NumPy View for MLP Feature Normalization | **High** | Logged (Iter 1) |
| **J-02** | 2026-07-26 | Architecture | Motion-Projected Centroid Association for Cross-Path Tracking | **High** | Logged (Iter 1) |
| **J-03** | 2026-07-26 | Performance | ONNX Runtime FP16 / ARM NEON Vectorized Model Execution | **Medium-High** | Logged (Iter 1) |
| **J-04** | 2026-07-26 | Sensor Fusion | Two-Stage Decoupled Vision and MLP Inference Pipeline | **High** | Logged (Iter 1) |
| **J-05** | 2026-07-26 | Architecture | Closest Point of Approach (CPA) Gating for TTC Speed Scaling | **High** | Logged (Iter 2) |
| **J-06** | 2026-07-26 | Performance | Vectorized Circular NumPy Buffer for Sliding Window Feature Extraction | **High** | Logged (Iter 2) |
| **J-07** | 2026-07-26 | Sensor Fusion | Intrinsic Camera Calibration & Distortion Correction for Centroids | **High** | Logged (Iter 2) |
| **J-08** | 2026-07-26 | Architecture | Tracking Confidence Gating for Navigation Speed Scaling | **High** | Logged (Iter 2) |
| **J-09** | 2026-07-26 | Sensor Fusion | Direct Hardware-Level Spatial Detection Intake to Bypass Host Vision CPU | **High** | Logged (Iter 3) |
| **J-10** | 2026-07-26 | Architecture | Backward Linear Extrapolation Padding for Instantaneous Track Initialization | **High** | Logged (Iter 3) |
| **J-11** | 2026-07-26 | Architecture | Euclidean Norm Acceleration Clamping with True Time-Delta Scaling | **High** | Logged (Iter 3) |
| **J-12** | 2026-07-26 | Performance | True Zero-Copy Pagelocked (Pinned) Host Buffers for Async TensorRT DMA | **High** | Logged (Iter 3) |
| **J-13** | 2026-07-26 | Architecture | Dynamic Predictive Control Barrier Function (DP-CBF) with Obstacle Velocity Drift | **High** | Logged (Iter 4) |
| **J-14** | 2026-07-26 | Architecture | Global Coordinate Frame Velocity Export for Rotational-Invariant Prediction | **High** | Logged (Iter 4) |
| **J-15** | 2026-07-26 | Architecture | Virtual PointCloud Forward-Projection Topic for Zero-Plugin Nav2 Integration | **High** | Logged (Iter 4) |
| **J-16** | 2026-07-26 | Performance | Asymmetric Kinematic Rate Limiter for Speed Scale Transitions | **High** | Logged (Iter 4) |
| **J-17** | 2026-07-26 | Architecture | Nav2 Action Feedback Telemetry Harvesting for Dynamic Replanning Triggers | **High** | Logged (Iter 5) |
| **J-18** | 2026-07-26 | Sensor Fusion | Ring-Buffered Full-Horizon LiDAR Sector Accumulation for Sub-Scan Latency | **High** | Logged (Iter 5) |
| **J-19** | 2026-07-26 | Architecture | Dynamic Approach-Gated MLP Horizon to Capture High-Speed Hazards Beyond 1.8m | **High** | Logged (Iter 5) |
| **J-20** | 2026-07-26 | Sensor Fusion | Modal Cluster Depth Extraction via 1D Histogram Density Peak Matching | **High** | Logged (Iter 5) |
| **J-21** | 2026-07-26 | Sensor Fusion | IMU-Odometry Residual Fusion for Real-Time Mecanum Lateral Slip Compensation | **High** | Logged (Iter 6) |
| **J-22** | 2026-07-26 | Architecture | Decoupled Kinematic Acceleration Clamping from Confidence Gating | **High** | Logged (Iter 6) |
| **J-23** | 2026-07-26 | Performance | Direct GPU CUDA Preprocessing Kernel for YOLOv8/v10 Input Normalization | **High** | Logged (Iter 6) |
| **J-24** | 2026-07-26 | Performance | Lock-Free Triple-Buffering Swap Chain for Zero-Copy Depth Frame IPC | **High** | Logged (Iter 6) |
| **J-25** | 2026-07-26 | Architecture | Ego-Motion Invariant Feature Extraction via Instantaneous Body-Frame Back-Projection | **High** | Logged (Iter 7) |
| **J-26** | 2026-07-26 | Architecture | True Distance-Normalized Pure Pursuit Curvature with MLP Interception Lookahead | **High** | Logged (Iter 7) |
| **J-27** | 2026-07-26 | Sensor Fusion | Hardware Subpixel Disparity Enablement for Medium/Long-Range Quantization | **High** | Logged (Iter 7) |
| **J-28** | 2026-07-26 | Architecture | Ego-Velocity Compensation in Travel-Aligned Relative Velocity Vectors for TTC | **High** | Logged (Iter 7) |

---

## 2. Architecture & Algorithmic Enhancements
*Focus areas: Nav2/MPPI integration, Kalman filtering, state machine transitions, predictive costmap layers, and domain adaptation.*

<!-- Add new architectural ideas below this line -->

### Idea J-25: Ego-Motion Invariant Feature Extraction via Instantaneous Body-Frame Back-Projection
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Moderate coding effort, eliminates false-positive obstacle approach velocity spikes during robot driving/turning)
- **Problem:** In `src/velocity_estimator.py` (`_build_window_features`, lines 380–417), the input feature matrix is assembled directly from `track['history']`, which records centroid coordinates $(cx, cy, cz)$ in the local camera/body frame at the historical instant each frame was captured ($t - k \Delta t$). When the robot drives forward at $0.3\text{ m/s}$ or turns, the apparent local coordinates of a completely stationary obstacle change over the 10-frame window ($1.0\text{ s}$). Consequently, the feature builder interprets robot ego-motion as obstacle approach velocity, causing the MLP to predict false $-0.3\text{ m/s}$ approach velocities on stationary furniture or standing pedestrians whenever the robot moves.
- **Proposed Solution:** Utilize `track['history_global']` (world-frame centroids) and project all 10 historical global centroids into the robot's **current instantaneous body frame at time $t_{\text{now}}$** (using `robot_pose_fn`) before passing them to `_build_window_features`. This ensures stationary obstacles have zero relative displacement ($\Delta x = 0, \Delta y = 0$) across the window regardless of how the robot moved during the last second.
- **Expected Benefit:** Achieves 100% ego-motion invariance for MLP feature vectors, preventing robot driving and turning from triggering false obstacle velocity predictions and emergency braking.

### Idea J-26: True Distance-Normalized Pure Pursuit Curvature with MLP Interception Lookahead
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Low coding effort, prevents close-range steering oscillations and under-steering)
- **Problem:** In `src/navigation_fsm.py` (`_handle_pure_pursuit`, line 470), curvature steering adjustment is calculated using `steering = -np.sin(bearing) * self.config.curvature_gain * 0.5`. In standard geometric Pure Pursuit, curvature $\kappa = \frac{2 \sin(\alpha)}{l_d}$, where $l_d$ is the lookahead distance (`distance`). By omitting division by `distance`, the steering command remains weakly identical whether the target is 3.0 meters away or 20 centimeters away, causing severe under-steering and oscillations as the robot approaches a goal. Furthermore, the FSM currently aims at the target's instantaneous position rather than anticipating its trajectory.
- **Proposed Solution:** Correct the steering curvature formula to include distance normalization: $\kappa = \frac{2 \sin(\alpha)}{d}$. When tracking moving pedestrians or dynamic waypoints, replace the instantaneous target position with the MLP's predicted interception lookahead point $(x + v_x \tau, y + v_y \tau)$.
- **Expected Benefit:** Enables smooth, mathematically rigorous arc trajectories that tighten dynamically at close range and intercept moving goals without pivoting or hunting.

### Idea J-28: Ego-Velocity Compensation in Travel-Aligned Relative Velocity Vectors for True Predictive TTC Braking
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (Low coding effort, enables proactive deceleration before breaching static proximity boundaries)
- **Problem:** In `src/ab_comparison_test.py` (`_get_speed_scaling`, lines 814–844), Time-to-Collision (TTC) is calculated using the dot product `r_dot_v = rx_t * rvx_t + ry_t * rvy_t`, where `rvx_t, rvy_t` is the obstacle's velocity vector predicted by the MLP in the robot frame. For a stationary obstacle or an obstacle moving slowly away, `rvx_t` is zero or positive, making `r_dot_v >= 0`. As a result, predictive TTC scaling (`s_t`) is completely bypassed even when the robot is driving rapidly toward the obstacle at $V_{\text{rob}} = 0.5\text{ m/s}$. The robot remains blind to its own approach velocity, relying solely on reactive emergency braking once the static 1.8m boundary is breached.
- **Proposed Solution:** Subtract the robot's commanded linear velocity vector $\mathbf{v}_{\text{rob}} = (vx_{\text{cmd}}, vy_{\text{cmd}})$ from the obstacle's velocity vector to compute true relative approach velocity $\mathbf{v}_{\text{rel}} = \mathbf{v}_{\text{obs}} - \mathbf{v}_{\text{rob}}$ before evaluating the travel-aligned dot product `r_dot_v`.
- **Expected Benefit:** Activates predictive TTC deceleration whenever the robot is on a collision course with an obstacle, ensuring smooth, human-friendly slowdowns from 3.0 seconds away.

### Idea J-22: Decoupled Kinematic Acceleration Clamping from Confidence Gating
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Low coding effort, eliminates up to 500 ms of velocity prediction lag for emerging runners)
- **Problem:** In `src/velocity_estimator.py` (`_inference_loop`, lines 584–601), predicted velocities are multiplied by tracking confidence (`vx * conf`) before applying the $3.0\text{ m/s}^2$ kinematic acceleration rate limiter (`max_delta = 0.3 m/s` per frame). When a newly detected runner ($2.5\text{ m/s}$) gains confidence over their first 10 frames (e.g., from `conf = 0.3` to `conf = 1.0`) or recovers from a brief occlusion, the rate limiter interprets the confidence-driven rise in `vx * conf` as physical human acceleration. Because $\Delta v > 0.3\text{ m/s}$ per frame during confidence ramp-ups, the rate limiter artificially clamps the output, delaying the true velocity report by 300–500 ms and corrupting the physical velocity state stored in `self._prev_estimates`.
- **Proposed Solution:** Decouple kinematic rate limiting from confidence scaling. Apply the $3.0\text{ m/s}^2$ acceleration clamp directly to the raw, unscaled MLP velocity prediction ($v_{\text{raw}}$) and update `self._prev_estimates` with the physical clamped velocity. Then multiply by `conf` as the final step before exporting to `estimates`: `vx_export = vx_clamped * conf`.
- **Expected Benefit:** Completely prevents tracking confidence transients from triggering false kinematic acceleration damping, delivering immediate, accurate velocity tracking for high-speed emerging pedestrians.

### Idea J-17: Nav2 Action Feedback Telemetry Harvesting for Dynamic Replanning Triggers
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Low coding effort, bridges Nav2 recovery struggles directly with predictive pedestrian tracking)
- **Problem:** In `src/nav2_client.py` (`_feedback_cb`, lines 271–275), the action client only extracts `distance_remaining` from the Nav2 action feedback message, ignoring critical telemetry such as `number_of_recoveries` and `estimated_time_remaining`. When a dynamic pedestrian obstructs the robot's corridor, Nav2 repeatedly triggers recovery behaviors (e.g., spinning in place or clearing costmaps) without informing the supervisor, eventually causing mission aborts (`STATE_FAILED`).
- **Proposed Solution:** Extract `number_of_recoveries` and `estimated_time_remaining` in `_feedback_cb` and expose them in `get_status()`. In `server_x3.py`, trigger an automatic dynamic replanning routine whenever `number_of_recoveries >= 2`: query `VelocityEstimator` for active pedestrian vectors, temporarily pause navigation for 1.5 seconds if a pedestrian is crossing, or laterally shift the intermediate waypoint by $0.8\text{m}$ away from the obstacle's velocity vector.
- **Expected Benefit:** Prevents Nav2 recovery loops and mission aborts in dynamic environments by dynamically coordinating global waypoint adjustment with real-time pedestrian velocity vectors.

### Idea J-19: Dynamic Approach-Gated MLP Horizon to Capture High-Speed Hazards Beyond 1.8m
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Low effort, ensures early detection and braking against fast runners without wasting CPU on background clutter)
- **Problem:** In `src/velocity_estimator.py` (`_inference_loop`, lines 484–496), tracks with centroid $z > 1.8\text{m}$ are bypassed and assigned $v_x = 0.0, v_y = 0.0$ to save CPU inference cycles on distant objects. For a pedestrian running towards the robot from $z = 2.5\text{m}$ at $2.0\text{ m/s}$ ($\text{TTC} = 1.25\text{s}$, well within the 3.0s emergency threshold), the zeroed velocity causes downstream TTC formulas ($\mathbf{r} \cdot \mathbf{v} = 0$) to compute infinite TTC. The robot maintains full speed ($0.5\text{ m/s}$) until the runner crosses the $1.8\text{m}$ line 0.35s later, leaving insufficient distance to decelerate safely.
- **Proposed Solution:** Replace the hard static $1.8\text{m}$ cutoff with a dynamic approach-gated horizon out to $R_{\text{far}} = 4.0\text{m}$ (full sensor range). For tracks between $1.8\text{m}$ and $4.0\text{m}$, evaluate the 2-frame depth difference: if $\Delta z / \Delta t < -0.3\text{ m/s}$ (active approach), execute full MLP inference and export true velocity; otherwise, skip inference for stationary background objects.
- **Expected Benefit:** Extends predictive safety horizon to 3.5 seconds for high-speed oncoming hazards while preserving low CPU utilization for stationary background clutter.

### Idea J-13: Dynamic Predictive Control Barrier Function (DP-CBF) with Obstacle Velocity Drift
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Low coding effort, transforms static safety barriers into proactive dynamic collision avoidance)
- **Problem:** In `src/cbf_filter.py` (`HolonomicCBFFilter.filter_velocity`, lines 45–61), the barrier function derivative $\dot{h} = 2(r_x - o_x)u_x + 2(r_y - o_y)u_y$ assumes all obstacles are completely stationary ($\dot{o}_x = 0, \dot{o}_y = 0$). When a dynamic pedestrian runs towards the robot at $1.5\text{ m/s}$, the static CBF constraint underestimates the gap closure rate and intervenes too late. Conversely, when a pedestrian moves away, the static CBF unnecessarily brakes or diverts the robot. Furthermore, `scipy.optimize.minimize(..., method='SLSQP')` is called at runtime, incurring 3–5 ms of Python overhead per frame for a simple 2D inequality QP.
- **Proposed Solution:** Pass estimated obstacle velocities $(v_{ox}, v_{oy})$ into `filter_velocity` and include the obstacle drift term in the CBF inequality constraint: $\nabla h \cdot \mathbf{u} - 2\big((r_x - o_x)v_{ox} + (r_y - o_y)v_{oy}\big) + \gamma h \ge 0$. Replace the SLSQP optimizer with an analytical 2D active-set projection to solve the quadratic objective in <0.05 ms.
- **Expected Benefit:** Prevents boundary violations against oncoming pedestrians, eliminates false-positive diversions for retreating pedestrians, and reduces CBF computation latency by 98%.

### Idea J-14: Global Coordinate Frame Velocity Export for Rotational-Invariant Prediction
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Low effort, essential for correct global trajectory prediction during robot turns)
- **Problem:** In `src/velocity_estimator.py` (`_inference_loop`, lines 529–534 and 579–587), historical global centroid positions are projected into the robot's instantaneous body frame at time $t_{\text{now}}$ before being fed to the MLP. As a result, the model predicts velocity $(v_x, v_y)$ relative to the robot's moving axes and only exports those body-frame components. When the robot turns rapidly ($\omega > 1.0\text{ rad/s}$), the exported velocity vector rotates with the chassis even if the pedestrian walks in a straight line in the world, causing global motion planners to predict circular, invalid trajectories.
- **Proposed Solution:** In `_inference_loop`, apply the inverse 2D rotation matrix using the robot's global yaw $\theta_{\text{rob}}$ to transform predicted body velocities back into world coordinates: $v_{G_x} = v_x \cos\theta - v_y \sin\theta$ and $v_{G_y} = v_x \sin\theta + v_y \cos\theta$. Export `'vx_global'` and `'vy_global'` in the estimate dictionary.
- **Expected Benefit:** Delivers rotational-invariant world velocity vectors to Nav2, MPPI, and costmap layers, enabling stable linear trajectory prediction while the robot maneuvers.

### Idea J-15: Virtual PointCloud Forward-Projection Topic for Zero-Plugin Nav2 Costmap Integration
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Moderate effort, achieves predictive obstacle avoidance in Nav2 without custom C++ plugins)
- **Problem:** When `FrontierExplorer` (or `Nav2Client`) navigates via standard Nav2, moving pedestrians appear only as instantaneous static obstacles on the local costmap, leaving a trail of stale occupied cells while failing to mark where the pedestrian is heading next.
- **Proposed Solution:** Create a lightweight ROS2 publisher node (or integrate into `server_x3.py`) that subscribes to `VelocityEstimator` outputs and publishes a `sensor_msgs/msg/PointCloud2` on `/predictive_obstacles`. For each tracked obstacle $i$ with speed $>0.2\text{ m/s}$, project virtual points along its estimated velocity vector $(v_{G_x}, v_{G_y})$ at future timestamps $t \in \{0.5\text{s}, 1.0\text{s}, 1.5\text{s}, 2.0\text{s}\}$. Configure Nav2's local `ObstacleLayer` to subscribe to `/predictive_obstacles` with `clearing: true, marking: true`.
- **Expected Benefit:** Enables standard Nav2 MPPI/DWB controllers to automatically curve around future pedestrian trajectories before collision risks occur, achieving full predictive local planning without modifying existing C++ costmap code.

### Idea J-10: Backward Linear Extrapolation Padding for Instantaneous Track Initialization
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Low coding investment, eliminates 0.7s velocity under-prediction lag for newly detected pedestrians)
- **Problem:** In `src/velocity_estimator.py` (`_build_window_features`, lines 375–378), when a new track has fewer than 10 frames (`len(hist) < WINDOW_SIZE`), it is padded using zero-order hold (`hist.insert(0, hist[0])`). For a newly detected pedestrian with 3 valid frames, this duplicates their initial position 7 times. Since translation normalization subtracts `hist[0]`, the first 7 frames have $\Delta x = 0, \Delta y = 0$. The MLP interprets this as a stationary object that suddenly moved, severely under-predicting instantaneous speed (by 60–80%) during the critical first 0.7 seconds of detection.
- **Proposed Solution:** When $2 \le K < 10$ valid frames exist, compute the average displacement vector $\Delta \mathbf{s}_{\text{avg}} = \frac{\mathbf{s}_{K-1} - \mathbf{s}_0}{K-1}$. Back-project missing historical frames $j < 0$ using linear extrapolation: $\mathbf{s}_j = \mathbf{s}_0 + j \cdot \Delta \mathbf{s}_{\text{avg}}$.
- **Expected Benefit:** Provides consistent non-zero displacement features across the entire 10-frame window from Frame 2 onwards, allowing the MLP to instantly output accurate velocity predictions without startup lag.

### Idea J-11: Euclidean Norm Acceleration Clamping with True Time-Delta Scaling
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Low coding effort, prevents trajectory warping and respects physical human kinematic limits)
- **Problem:** In `_inference_loop` (lines 589–601), velocity changes between frames are clamped independently along each axis using `if abs(dvx) > max_delta: ...` where `max_delta = 3.0 / INFER_HZ`. Independent box-clamping allows diagonal acceleration up to $\sqrt{0.3^2 + 0.3^2} = 0.424\text{ m/s per frame}$ ($4.24\text{ m/s}^2$, violating the $3.0\text{ m/s}^2$ limit by 41%) and warps the trajectory direction when one axis is clipped while the other is not. Additionally, hardcoding `INFER_HZ` causes artificial slowdowns if vision processing spikes delay the loop.
- **Proposed Solution:** Calculate the Euclidean velocity change magnitude $\|\Delta \mathbf{v}\| = \sqrt{\Delta v_x^2 + \Delta v_y^2}$ and dynamically scale by actual elapsed time $\Delta t_{\text{actual}}$. If $\|\Delta \mathbf{v}\| > 3.0 \cdot \Delta t_{\text{actual}}$, scale both components uniformly by $\frac{3.0 \cdot \Delta t_{\text{actual}}}{\|\Delta \mathbf{v}\|}$.
- **Expected Benefit:** Preserves exact trajectory angles during rapid acceleration, strictly enforces circular kinematic limits, and adapts smoothly to framerate jitter.

### Idea J-05: Closest Point of Approach (CPA) Gating for TTC Speed Scaling
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Low coding investment, eliminates false emergency stops when pedestrians walk past without colliding)
- **Problem:** In `src/ab_comparison_test.py` (`_get_speed_scaling`, lines 837–846), Time-to-Collision (TTC) is calculated using the 1D projection formula $\text{TTC} = -d^2 / (\mathbf{r} \cdot \mathbf{v})$. If an obstacle has a negative dot product ($\mathbf{r} \cdot \mathbf{v} < 0$), this formula assumes a direct collision course even if the pedestrian is walking parallel to the robot's path (e.g., 1.5m to the left) and will pass safely without ever entering the collision cylinder.
- **Proposed Solution:** Compute the true 2D Distance-at-Closest-Approach (DCA) and Time-to-Closest-Approach (TCA):
  $$t_{cpa} = -\frac{\mathbf{r} \cdot \mathbf{v}_{rel}}{\|\mathbf{v}_{rel}\|^2}, \quad d_{cpa} = \|\mathbf{r} + t_{cpa} \mathbf{v}_{rel}\|$$
  Only apply TTC speed reduction if $t_{cpa} > 0$ and $d_{cpa} < R_{\text{robot}} + R_{\text{obs}} + 0.3\text{m}$.
- **Expected Benefit:** Prevents false-positive braking when dynamic pedestrians cross or pass the robot's lateral corridor safely.

### Idea J-08: Tracking Confidence Gating for Navigation Speed Scaling & Bypass Decisions
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Very low effort, significantly improves navigation smoothness in noisy depth environments)
- **Problem:** Currently, `VelocityEstimator` scales predicted velocity by tracking confidence (`conf = min(1.0, visible_count / 10)`), but it does not export `conf` or `visible_count` in the estimate dictionary. When `_get_speed_scaling()` in `ab_comparison_test.py` evaluates obstacle proximity and TTC, every detection is treated with equal weight. Brief 2-frame depth noise spikes or flickering reflections trigger immediate emergency proximity braking ($s_p \to 0$).
- **Proposed Solution:** Export `'conf': round(conf, 2)` and `'visible_count': visible_count` in `VelocityEstimator.get_estimates()`. In `_get_speed_scaling()` and camera bypass logic, ignore obstacles with `conf < 0.4` for emergency braking unless confirmed by LiDAR, and modulate TTC brake severity by `s_obs_weighted = 1.0 - conf * (1.0 - s_obs)`.
- **Expected Benefit:** Insulates the holonomic navigation loop from transient depth sensor spikes and visual tracking noise.

### Idea J-02: Motion-Projected Centroid Association for Cross-Path Tracking
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Low coding investment, major accuracy improvement in dynamic multi-pedestrian scenarios)
- **Problem:** Currently, `ObstacleTracker.update()` in `src/velocity_estimator.py` matches new detections to existing tracks purely by Euclidean distance (`np.linalg.norm`) against the static previous global centroid `(tx, ty)` using a fixed threshold `max_dist = 0.8m`. When fast-moving pedestrians cross paths or pass close to one another during a temporary 1-frame occlusion or depth dropout, static proximity matching frequently causes identity switching or merges distinct tracks.
- **Proposed Solution:** Utilize the MLP's latest estimated velocity `(last_vx, last_vy)` for each active track to forward-project its expected global position before matching:
  $$\hat{x} = t_x + v_x \cdot \Delta t, \quad \hat{y} = t_y + v_y \cdot \Delta t$$
  Compute the distance matrix against these motion-projected coordinates $(\hat{x}, \hat{y})$ rather than static coordinates $(t_x, t_y)$.
- **Expected Benefit:** Eliminates identity switching during intersecting pedestrian trajectories and reduces tracking dropouts without adding complex Kalman filter prediction state overhead.

---

## 3. Performance & Execution Efficiency
*Focus areas: TensorRT acceleration, zero-copy shared memory IPC, asyncio event loop optimization, and memory allocation reduction.*

<!-- Add new performance ideas below this line -->

### Idea J-23: Direct GPU CUDA Preprocessing Kernel for YOLOv8/v10 Input Normalization
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Moderate effort, eliminates ~7 ms host CPU preprocessing bottleneck per frame)
- **Problem:** In `src/trt_detector.py` (`_preprocess`, line 179), `cv2.dnn.blobFromImage` is used on the host CPU to resize BGR frames to $640 \times 640$, convert BGR to RGB, normalize pixels by $/255.0$, and transpose to NCHW format. This allocates an intermediate 1.2 MB float32 array (`blob`) and consumes ~6–8 ms of host CPU time before copying the result to `self._h_input`. On the Jetson Orin Nano's Unified Memory Architecture (UMA), performing pixel normalization and layout transformation on the CPU wastes precious host cycles while leaving the GPU idle before inference.
- **Proposed Solution:** Implement a lightweight 15-line PyCUDA elementwise kernel (or a custom TensorRT input layer) that reads the raw uint8 BGR image directly from UMA memory and performs color swapping, resizing, and float32 scaling directly on the GPU's CUDA cores.
- **Expected Benefit:** Reduces preprocessing latency from ~7 ms down to $<0.1\text{ ms}$ and frees up 100% of host CPU cycles for the MLP velocity estimator and Nav2 local planner.

### Idea J-24: Lock-Free Triple-Buffering Swap Chain for Zero-Copy Depth Frame IPC
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Low effort, eliminates thread tearing and mutex contention in shared depth frame IPC)
- **Problem:** In `src/server_x3.py` (`_depth_cb`, lines 407–415), incoming depth frames are copied directly into a single global numpy buffer `_shared_depth_array` (`np.copyto(_shared_depth_array, arr_meters)`), and `self._latest_raw_depth` is set to point to this exact buffer. When `VelocityEstimator` or `OakDCamera` calls `get_raw_depth_frame()` concurrently, they read from `_shared_depth_array` while the ROS2 thread is actively overwriting it via `np.copyto`. This causes severe frame tearing (where half the image is from frame $N$ and half from $N+1$) and can expose transient `NaN`/`inf` values during copy operations, corrupting 3D centroid localization.
- **Proposed Solution:** Implement a lock-free triple-buffering swap chain (`_buf_write`, `_buf_ready`, `_buf_read`). In `_depth_cb`, write incoming data into `_buf_write` and atomically swap pointers with `_buf_ready`. In `get_raw_depth_frame()`, atomically swap `_buf_read` with `_buf_ready` and return `_buf_read`.
- **Expected Benefit:** Achieves zero-copy, zero-allocation depth frame sharing while guaranteeing 100% tear-free image data and eliminating mutex blocking between the ROS2 thread and inference engines.

### Idea J-16: Asymmetric Kinematic Rate Limiter for Speed Scale Transitions
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 4)
- **ROI Tier:** **High ROI** (Low effort, prevents wheel slip and camera pitch-induced false positive braking loops)
- **Problem:** In `src/ab_comparison_test.py` (`_get_speed_scaling`, lines 850–857), when `speed_scale` drops, `smooth_speed_scale` drops instantly to the new lower value (step-function braking). Conversely, when `speed_scale` rises, it recovers using a fixed exponential moving average ($\beta = 0.15$ per loop). Step-function deceleration ($>6.0\text{ m/s}^2$) causes tire-to-ground static friction failure (wheel slip), which invalidates odometry and corrupts LiDAR scan-matching ICP. Furthermore, the chassis pitching during hard braking tilts the camera downward, causing it to detect the floor as an obstacle at $z = 0.8\text{m}$ and trapping the robot in a false-positive braking state.
- **Proposed Solution:** Implement an asymmetric kinematic rate limiter with true time-delta ($\Delta t$) scaling. Clamp the maximum deceleration rate to $a_{\text{brake, max}} = 1.5\text{ m/s}^2$ and recovery acceleration to $a_{\text{accel, max}} = 0.8\text{ m/s}^2$. Update the scale via `smooth_speed_scale = np.clip(target_scale, prev_scale - (1.5 * dt)/v_nom, prev_scale + (0.8 * dt)/v_nom)`.
- **Expected Benefit:** Eliminates wheel slip, maintains LiDAR ICP alignment, prevents chassis pitch-induced camera false positives, and ensures framerate-independent speed recovery.

### Idea J-12: True Zero-Copy Pagelocked (Pinned) Host Buffers for Async TensorRT DMA
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Moderate effort, saves ~2 ms per frame and unlocks true asynchronous GPU DMA transfers)
- **Problem:** In `src/trt_detector.py` (line 162), `self._h_input` is allocated with standard `np.empty()`, creating pageable memory. When `self._cuda.memcpy_htod_async(...)` is called, the CUDA driver must implicitly stage the transfer through a hidden temporary pagelocked bounce buffer and synchronize the CPU thread, defeating async execution. Furthermore, in `_preprocess()`, `cv2.dnn.blobFromImage` allocates an intermediate 1.2 MB float32 array on every frame only to copy it into `self._h_input`.
- **Proposed Solution:** Allocate host buffers using `pycuda.driver.pagelocked_empty(in_shape, dtype=np.float32)` for pinned memory. Replace `blobFromImage` array creation with direct channel scaling into the pagelocked buffer (e.g., using `np.multiply(resized_frame[:, :, ch], 1/255.0, out=self._h_input[0, 2-ch])` or passing destination buffers).
- **Expected Benefit:** Eliminates 1.2 MB per-frame CPU memory allocation/copying and enables true zero-copy DMA over the Jetson Orin Nano's unified memory architecture.

### Idea J-06: Vectorized Circular NumPy Buffer for Sliding Window Feature Extraction
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Low coding investment, eliminates 100+ intermediate Python list/array allocations per second)
- **Problem:** For every tracked obstacle on each inference cycle, `_inference_loop` and `_build_window_features` in `src/velocity_estimator.py` convert track history deques to lists of tuples, allocate multiple intermediate numpy arrays (`(T, 3)`, `(T, 2)`), perform matrix multiplications, and iterate in Python loops with list comprehensions to compute successive frame displacements ($\Delta x, \Delta y$).
- **Proposed Solution:** Replace the track deque with a pre-allocated circular 2D NumPy array `self.history_buf = np.zeros((WINDOW_SIZE, 3), dtype=np.float32)`. Use vectorized slicing (`diffs = np.diff(local_xy, axis=0)`) and in-place clamping (`np.clip(diffs, -0.25, 0.25, out=diffs)`) to generate the 40-dimensional feature vector without Python loop overhead or tuple conversions.
- **Expected Benefit:** Eliminates garbage collector churn and reduces CPU feature extraction time per track by ~70%.

### Idea J-01: Zero-Copy Pre-Allocated NumPy View for MLP Feature Normalization
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Extremely low investment, immediate reduction in CPU garbage collection and allocation overhead)
- **Problem:** In `src/velocity_estimator.py` (`_inference_loop`, lines 557–562), the feature matrix is normalized by creating an intermediate array `features_scaled`, which is then wrapped in a temporary PyTorch tensor via `torch.from_numpy(features_scaled)` and copied into `self.x_tensor_preallocated` via `.copy_()`. This instantiates temporary Python objects and performs redundant memory copies at 10Hz for every inference cycle.
- **Proposed Solution:** In `__init__`, expose a zero-copy NumPy view of the pre-allocated PyTorch tensor: `self.x_numpy_view = self.x_tensor_preallocated.numpy()`. In `_inference_loop`, perform standard scaler subtraction and multiplication in-place directly into this view using `np.subtract(features_batch, self.scaler_X_mean, out=self.x_numpy_view[:num_tracks])` and `np.multiply(self.x_numpy_view[:num_tracks], self.scaler_X_inv_scale, out=self.x_numpy_view[:num_tracks])`.
- **Expected Benefit:** Bypasses intermediate array allocations, temporary Tensor object creation, and the `.copy_()` memory transfer entirely, reducing CPU load and heap fragmentation on the Jetson Orin Nano.

### Idea J-03: ONNX Runtime FP16 / ARM NEON Vectorized Model Execution
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 1)
- **ROI Tier:** **Medium-High ROI** (Moderate effort, 3x–5x reduction in inference forward-pass latency)
- **Problem:** `VelocityEstimator` executes its 3-layer MLP via PyTorch CPU TorchScript JIT tracing (`torch.jit.trace`) in FP32 precision. On ARM CPUs (like the Jetson Orin Nano Cortex-A78AE), PyTorch CPU JIT incurs significant Python C++ binding overhead and suboptimal vectorization for small batch sizes ($N \le 5$).
- **Proposed Solution:** Export the trained MLP weights to ONNX format (`velocity_mlp.onnx`) and replace `torch.jit.load` with `onnxruntime.InferenceSession(..., providers=['CPUExecutionProvider'])` using FP16 weights or int8 dynamic quantization.
- **Expected Benefit:** ONNX Runtime's ARM NEON CPU backend reduces small-MLP forward-pass latency from ~1.5–2.0 ms down to <0.3 ms per batch while lowering RAM consumption by ~40 MB.

---

## 4. Sensor Fusion & Hardware Integration
*Focus areas: OAK-D vs. Astra Pro depth calibration, YDLidar X3 mounting/scan-matching, IMU slip compensation, and motor driver telemetry.*

<!-- Add new hardware/sensor ideas below this line -->

### Idea J-27: Hardware Subpixel Disparity Enablement for Medium/Long-Range Quantization Velocity Spikes
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 7)
- **ROI Tier:** **High ROI** (One-line configuration change, eliminates 53 cm depth quantization jumps at 3.5m–4.5m range)
- **Problem:** In `src/oakd_driver.py` (line 232), `StereoDepth` is initialized with `stereo.setSubpixel(False)`. Without subpixel disparity interpolation, disparity is quantized to integer pixel values. On the OAK-D Lite (7.5 cm baseline, 400P resolution), integer disparity changes from 8 pixels ($3.75\text{ m}$) to 7 pixels ($4.28\text{ m}$) in a single step—a discrete **53 cm depth quantization jump** between adjacent pixel disparities. When a pedestrian walking at $1.0\text{ m/s}$ crosses this boundary, the instantaneous 53 cm depth snap in $0.1\text{ s}$ generates an artificial $+5.3\text{ m/s}$ velocity spike (19 km/h acceleration), causing the MLP velocity estimator to output erratic speed surges.
- **Proposed Solution:** Enable hardware subpixel disparity interpolation by setting `stereo.setSubpixel(True)` in `_build_pipeline()`. The OAK-D's on-device Myriad X ASIC performs 3-bit fractional disparity interpolation (1/8th pixel precision) at 30 Hz with zero host CPU overhead.
- **Expected Benefit:** Refines depth resolution at $3.75\text{ m}$ from a 53 cm quantization jump down to $6.6\text{ cm}$, delivering smooth, continuous centroid trajectories and eliminating false velocity spikes at medium and long ranges.

### Idea J-21: IMU-Odometry Residual Fusion for Real-Time Mecanum Lateral Slip Compensation
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 6)
- **ROI Tier:** **High ROI** (Moderate effort, eliminates 15–30 cm lateral drift during point-to-point and waypoint navigation)
- **Problem:** In `src/point_to_point_test.py` (lines 154–157) and `src/ab_comparison_test.py`, the robot relies on wheel odometry (`/odom`) to maintain straight-line navigation. On the Yahboom X3's mecanum drivetrain, minor roller friction imbalances or carpet resistance cause uncommanded lateral crabbing and wheel slip. Wheel odometry assumes pure kinematic rolling without slip, so it reports straight-line progress while the robot physically drifts 15–30 cm sideways over a 4-meter path, corrupting waypoint tracking and LiDAR scan-matching alignment.
- **Proposed Solution:** Utilize the 6-axis IMU data ($g_z, a_y$) provided by `Rosmaster.get_imu_data()` at 100Hz in `src/drivers_x3.py`. Compute the real-time residual between kinematic odometry yaw rate $\dot{\theta}_{\text{odom}}$ and IMU gyro $g_z$, and fuse uncommanded lateral acceleration $a_y$ into an adaptive slip-compensation matrix that dynamically trims left/right motor velocity commands.
- **Expected Benefit:** Cancels out mecanum wheel slip and lateral crabbing in real-time, maintaining true straight-line trajectory tracking on imperfect flooring without relying solely on slow LiDAR ICP corrections.

### Idea J-18: Ring-Buffered Full-Horizon LiDAR Sector Accumulation for Sub-Scan Latency Reduction
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Moderate effort, eliminates 83% blind spots during low-latency sub-scan publishing)
- **Problem:** In `src/drivers_x3.py` (`YDLidarDriver._scan_loop`, lines 548–558), to reduce scan publishing latency, the driver flushes every $45^\circ$ sub-scan arc and overwrites `self._points` with only those points. Because $45^\circ$ represents just 1/6th of the $270^\circ$ active FOV, downstream odometry ICP scan-matching and obstacle avoidance see an incomplete map missing 83% of the surrounding environment at any instant. Pedestrians outside the active $45^\circ$ wedge disappear for 5 out of 6 cycles, causing ICP scan-matching failures and delayed avoidance.
- **Proposed Solution:** Implement a ring-buffered full-horizon pointcloud bucketed into 8 angular sectors ($45^\circ$ each). When a new sub-scan arc arrives for sector $k$, overwrite only sector $k$ in the ring buffer, then concatenate all valid sectors into `self._points`.
- **Expected Benefit:** Delivers immediate low-latency updates ($45^\circ$ sub-scans publish instantly without waiting 125 ms for a full rotation) while ensuring downstream ICP scan-matching and collision barriers always operate on a complete, dense $270^\circ$ environmental map.

### Idea J-20: Modal Cluster Depth Extraction via 1D Histogram Density Peak Matching for Occlusion-Robust 3D Centroids
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 5)
- **ROI Tier:** **High ROI** (Low effort, eliminates 47 km/h velocity spikes caused by partial foreground occlusions)
- **Problem:** In `src/oakd_driver.py` (`_locate`, line 568), the 3D depth of a person bounding box is computed using the 20th percentile of valid depth pixels within the inner 50% ROI (`np.percentile(valid, 20)`). When a foreground obstacle (e.g., table edge, chair, or another pedestrian's arm) intersects the bottom of the target person's bounding box, the 20th percentile grabs the depth of the foreground obstacle instead of the person. If a person at $z = 2.5\text{m}$ is partially occluded by a table at $z = 1.2\text{m}$, the depth estimate jumps by $-1.3\text{m}$ in one frame, generating an artificial $-13.0\text{ m/s}$ velocity spike that triggers false emergency braking.
- **Proposed Solution:** Replace percentile thresholding with 1D histogram density peak matching (KDE mode tracking). Build a histogram of depth values in the ROI with 10 cm bins, identify the primary density mode within $[z_{\text{prev}} - 0.5\text{m}, z_{\text{prev}} + 0.5\text{m}]$ (or the largest cluster if new), and compute median depth only within that modal cluster.
- **Expected Benefit:** Completely rejects foreground occlusion artifacts and background wall inflation, delivering clean, jitter-free 3D centroid trajectories to the MLP velocity estimator.

### Idea J-09: Direct Hardware-Level Spatial Detection Intake to Bypass Host Vision CPU Overhead
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 3)
- **ROI Tier:** **High ROI** (Low effort, eliminates 100% of host CPU morphology and contour processing when OAK-D is active)
- **Problem:** Currently, when `VelocityEstimator` receives depth from `OakDCamera`, `_extract_depth_centroids` in `src/velocity_estimator.py` executes CPU-intensive OpenCV morphological filtering (`cv2.morphologyEx`), contour finding, bounding box cropping, and median filtering on the raw 3D depth map. However, `OakDCamera` already runs on-device spatial YOLO detection (`get_spatial_detections()`), which directly computes high-confidence bounding boxes and 3D optical/base coordinates (`xyz_m` / `xyz_base_m`) on the hardware Neural Network and stereo depth engine.
- **Proposed Solution:** In `_extract_depth_centroids`, check if `self.detections_fn` (or `cam_src.get_spatial_detections`) returns valid spatial detections with `xyz_m`. If available, extract `(det["xyz_m"]["x"], det["xyz_m"]["y"], det["xyz_m"]["z"])` directly and bypass the OpenCV morphology and contour extraction loop entirely. Fall back to contour finding only when spatial detections are unavailable.
- **Expected Benefit:** Reduces vision processing latency from ~15–20 ms down to <1 ms per cycle on the Jetson Orin Nano while replacing fragmented depth contours with clean semantic person bounding boxes.

### Idea J-07: Intrinsic Camera Calibration & Distortion Correction for Centroids
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 2)
- **ROI Tier:** **High ROI** (Moderate effort, eliminates systematic 5–10 cm lateral position errors at FOV boundaries)
- **Problem:** In `_extract_depth_centroids`, pixel centroids $(c_x, c_y)$ are converted to physical meters using a simplified pinhole camera model with a hardcoded focal length (`fx = 277.0`) and optical center assumed at exactly $(160.0, 120.0)$. Real Orbbec Astra Pro and OAK-D lenses exhibit radial and tangential distortion ($k_1, k_2, p_1, p_2$), causing un-undistorted edge detections to suffer systematic lateral position errors. Over a 10-frame window, these spatial distortions create artificial lateral velocity vectors as pedestrians move across the FOV.
- **Proposed Solution:** Load exact camera intrinsic calibration matrices ($K, D$) from ROS `camera_info` or a YAML calibration file and apply `cv2.undistortPoints` to contour centroids prior to 3D back-projection.
- **Expected Benefit:** Removes optical distortion artifacts at FOV edges, preventing false lateral velocity predictions and improving cross-path trajectory estimation accuracy.

### Idea J-04: Two-Stage Decoupled Vision and MLP Inference Pipeline
- **Date Logged:** 2026-07-26 (Hourly Routine Iteration 1)
- **ROI Tier:** **High ROI** (Moderate effort, eliminates control loop jitter and frame dropping)
- **Problem:** Currently, `_inference_loop` synchronously fetches depth frames (`cam_src.get_depth_frame()`), executes CPU-intensive OpenCV morphological filtering, contour finding, and median depth extraction (`_extract_depth_centroids`), and then runs MLP inference sequentially in the same thread. During crowded scenes with large contours, depth extraction can spike to 15–25 ms, consuming a large portion of the 100 ms cycle budget (`INFER_HZ = 10`) and delaying velocity commands.
- **Proposed Solution:** Decouple depth processing from MLP inference into two independent pipeline stages:
  1. **Vision Worker Thread**: Runs `_extract_depth_centroids` at the camera's native framerate (15–20 Hz) and deposits timestamped `centroids_m` into an atomic, lock-free reference.
  2. **10Hz Control Loop**: Wakes up precisely every 100 ms, grabs the latest available centroids, performs global coordinate transformation, and executes MLP inference.
- **Expected Benefit:** Guarantees rigid 10Hz (`dt = 0.1s`) execution regularity for the velocity regression pipeline, insulating navigation control from camera frame-rate jitter or vision processing spikes.

---

## 5. Log & Prioritization Guidelines

To maintain document readability and prevent log bloat:
1. **Categorize Immediately**: Place new ideas under their respective domain section (Sections 2–4).
2. **Assign an ROI Tier**: Grade each idea based on implementation effort vs. runtime impact:
   - **High ROI**: Low investment (<2 hours), significant gains in CPU/RAM, safety, or accuracy.
   - **Medium ROI**: Moderate effort (half-day to 1 day), solid architecture or navigation benefits.
   - **Low ROI**: High effort or minor edge-case improvements.
3. **Monthly Archiving**: At the end of each month, move completed or historical ideas to the `ideas/` archive folder.
