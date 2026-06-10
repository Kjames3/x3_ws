# Project Improvement Ideas Log

This log tracks ideas and architectural enhancements for the EE244 Computational Learning Project (Predictive Local Planning via Onboard Velocity Estimation).

---

## [2026-06-04 02:32:00 -07:00] Initial Analysis

### 1. Ego-Motion Compensation in `VelocityEstimator`
* **Context:** In `server_x3.py`, `VelocityEstimator` is initialized with `robot_pose_fn=None`. Furthermore, the `VelocityEstimator` class itself stores this callback in its constructor but does not invoke it during its `_inference_loop` processing.
* **The Issue:** When the robot moves, static obstacles appear to shift in the camera frame. The MLP model (trained on absolute motion capture velocities) receives relative displacements (`dx = x - prev_x`) and incorrectly infers non-zero obstacle speeds.
* **Proposed Enhancement:**
  - Expose a thread-safe `get_pose_m()` (meters/radians) method from `ROS2Bridge` in `server_x3.py`.
  - Update `VelocityEstimator`'s inference loop to query this pose and transform local centroids `(x_rel, y_rel)` to the global map frame:
    $$x_{global} = x_{robot} + x_{rel}\cos(\theta) - y_{rel}\sin(\theta)$$
    $$y_{global} = y_{robot} + x_{rel}\sin(\theta) + y_{rel}\cos(\theta)$$
  - Perform tracking and sequence feature engineering (`dx`, `dy`) in the global frame to isolate the pedestrian's true velocity relative to the ground.

### 2. Live ROS2 Topic Publishing for Nav2/MPPI Integration
* **Context:** The velocity estimates are currently only sent to the Web GUI interface via a WebSocket JSON frame for audience visualization. The robot's onboard planner cannot access them.
* **The Issue:** The upcoming **Week 7 A/B controller demo** compares the MPPI planner with and without velocity estimates injected into the costmap. Without a ROS2 topic, the planner has no way of obtaining these estimates.
* **Proposed Enhancement:**
  - Create a new ROS2 publisher inside `ROS2Bridge` (e.g. topic `/pedestrian_states` utilizing standard `visualization_msgs/MarkerArray` or custom obstacle messages).
  - Publish the position, velocity vector, and predicted future position bounds of each tracked pedestrian.
  - Implement/configure a dynamic costmap inflation layer that subscribes to `/pedestrian_states` and updates cost grids ahead of the pedestrian's path, forcing MPPI to steer predictively.

### 3. PD-based Closed-Loop Heading Control in Calibration Nodes
* **Context:** Both `point_to_point_test.py` and `ab_comparison_test.py` use a purely proportional (P) controller for closed-loop heading rotations:
  `rot_speed = max(MIN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, abs(yaw_error) * KP_ROT))`
* **The Issue:** P-only controllers suffer from overshoot and oscillation near target headings ($180^\circ$ turns), requiring hardcoded boundary threshold checks to break tie-flips.
* **Proposed Enhancement:**
  - Add derivative tracking to implement a Proportional-Derivative (PD) control loop. The derivative term will act as an active dampener, slowing down angular rotation as heading error drops to zero, yielding smooth, overshoot-free settling.

---

## [2026-06-04 04:30:00 -07:00] Iteration 1 Analysis

### 4. Hybrid Kalman Filter Tracking & MLP Predictor Fusion
* **Context:** Currently, `velocity_estimator.py` uses a simple nearest-neighbor centroid association algorithm.
* **The Issue:** While simple and computationally lightweight, nearest-neighbor association suffers from identity switches and tracking dropouts under pedestrian crossovers, rapid accelerations, or partial occlusions. The network outputs are feed-forward and do not close the loop to assist the tracker.
* **Proposed Enhancement:**
  - Implement a **State-Space Kalman Filter** tracker where each track represents a state vector $x = [x, y, v_x, v_y]^T$.
  - In the prediction step, use the MLP's estimated velocity outputs as control inputs ($u_k$) to project the future state estimate:
    $$x_k^- = A x_{k-1} + B u_k$$
  - In the update step, fuse the raw centroid measurements from both the camera (depth contours) and the LiDAR (DBSCAN clusters) using a Kalman measurement update matrix.
  - Fusing predicted MLP velocities with raw spatial measurements will prevent track loss during temporary sensor dropouts.

### 5. Spatio-Temporal Trajectory Rollouts for Costmap Updates
* **Context:** The project's navigation system utilizes the Nav2 MPPI (Model Predictive Path Integral) local planner, which generates rollouts based on a static costmap layer.
* **The Issue:** Standard static costmaps only represent the human's current position as an obstacle. The planner lacks temporal foresight, leading to late evasive maneuvers or collision paths when pedestrians cross in front of the robot.
* **Proposed Enhancement:**
  - Implement a Python node that subscribes to `/pedestrian_states` (published from the estimator) and projects human trajectories over a $2.0\text{ second}$ horizon:
    $$x(t) = x_0 + v_x \cdot t$$
    $$y(t) = y_0 + v_y \cdot t$$
  - Publish these projected future positions as a series of inflated circle updates to Nav2's costmap update topics.
  - By adding time-decaying cost values to these future regions, MPPI's trajectory optimization will automatically select paths that go *behind* the pedestrian rather than cutting across their trajectory.

### 6. Auto-Tuning/Self-Calibration of Controller Gains
* **Context:** The proportional gains for the linear and rotation controllers (`KP_DIST`, `KP_YAW`, `KP_ROT`) in the test scripts are statically tuned.
* **The Issue:** Changes in battery voltage, wheel wear, or floor surface friction (e.g. carpet vs tile) can cause sluggish drive speeds or target heading overshoots.
* **Proposed Enhancement:**
  - Write an automated calibration routine run at the start of `point_to_point_test.py`.
  - The robot performs a brief $10\text{ cm}$ linear drive and $10^\circ$ rotational pulse, recording the actual acceleration, steady-state speed, and decay profiles.
  - Dynamic gains are then auto-calculated using a simple auto-tuning heuristic to ensure identical path profiles regardless of battery charge or surface friction.

---

## [2026-06-04 06:00:00 -07:00] Iteration 2 Analysis

### 7. Adaptive LiDAR Outlier Filtering in Indoor Classrooms
* **Context:** LiDAR scan parsing in `ydlidar_ros2_driver_node` currently forwards raw sensor feeds directly to the clustering pipeline.
* **The Issue:** Indoor deployment environments like classrooms contain multi-path reflections (specular noise from glossy whiteboard backdrops or metal furniture frames). These reflections produce phantom spatial outliers that distort the DBSCAN clustering step, creating ghost obstacles.
* **Proposed Enhancement:**
  - Introduce an online **LaserScanMedianFilter** node on the `/scan` topic to suppress isolated range spikes.
  - Apply range-rate thresholding to discard scan updates showing rapid, unphysical changes in sequential sweeps.

### 8. Dynamic Window Sizing for High-Acceleration Pedestrians
* **Context:** The features mapped to the MLP use a fixed sequence history window (`WINDOW_SIZE = 10` frames, representing $1.0\text{ second}$).
* **The Issue:** A static 1.0s window introduces lag when a pedestrian rapidly changes pace (e.g. stops abruptly or starts running). The MLP output remains weighted by older constant-velocity samples.
* **Proposed Enhancement:**
  - Implement a variable-length history queue that dynamically scales window size $T$ based on the estimated acceleration of the tracked centroid.
  - During periods of high acceleration, shrink the active window size to $T=5$ (0.5s of history) to prioritize immediate local changes and reduce prediction latency.

### 9. RViz2 Spatio-Temporal Trajectory & Velocity Vector Markers
* **Context:** The pedestrian velocity vectors are visualized on a custom web GUI page, but cannot be natively visualized in standard ROS2 diagnostic tools.
* **The Issue:** ROS2 development relies heavily on RViz2 for spatial debugging. Engineers cannot inspect tracking performance, predicted heading vectors, or planner cost boundaries alongside the map and TF trees.
* **Proposed Enhancement:**
  - Create a visualization node inside `server_x3.py` that publishes `visualization_msgs/MarkerArray` on `/pedestrian_markers`.
  - Publish colored velocity arrow markers (pointing in the direction of $v_x, v_y$ with length proportional to speed) and semi-transparent covariance ellipses showing the prediction uncertainty bounds over time.

---

## [2026-06-04 07:30:00 -07:00] Iteration 3 Analysis

### 10. Sensor Failure Fallback and Dynamic Source Re-routing
* **Context:** The `VelocityEstimator` is designed to process colorised depth frames from the camera to extract centroids.
* **The Issue:** A hardware glitch on a single sensor (e.g. depth camera USB connection failure or low power drop) will completely blind the velocity estimation pipeline, even if other sensors like LiDAR are still functioning.
* **Proposed Enhancement:**
  - Implement a **source routing fallback listener** inside the estimator.
  - If depth frames stop arriving (timeout > 2.0s), dynamically redirect the tracking pipeline to process raw 2D LiDAR clusters. Use DBSCAN clusters to extract centroids and feed the displacement sequences to the MLP model. This keeps the planner active (with slightly degraded spatial accuracy) rather than halting completely.

### 11. Encoders-IMU Slip Compensation using EKF Velocity Feedback
* **Context:** Subtracted robot speed is used to compensate for ego-motion on the tracked pedestrian coordinates.
* **The Issue:** Mecanum wheels are prone to slipping, especially during rapid accelerations or when strafing. Relying purely on raw wheel encoders to calculate the robot's motion leads to significant errors in the subtracted velocity vector, distorting the estimated pedestrian speed.
* **Proposed Enhancement:**
  - Feed the EKF-fused velocities directly from the `/odom` Twist topic (`msg.twist.twist.linear.x`, `msg.twist.twist.linear.y`, and `msg.twist.twist.angular.z`) into the coordinate subtraction calculations.
  - Since the EKF filters encoder ticks against IMU linear accelerometers and gyroscopes, this provides a highly accurate body velocity estimate that accounts for physical wheel slip.

### 12. Non-Linear Pedestrian Path Modeling via Spline Extrapolations
* **Context:** Trajectory rollouts for costmap inflation assume constant linear velocity ($x(t) = x_0 + v_x t$).
* **The Issue:** Humans rarely walk in straight lines inside classrooms; they navigate around tables and curve to avoid walls. Linear projections over longer horizons ($> 1.0\text{ second}$) lead to incorrect collision predictions.
* **Proposed Enhancement:**
  - Fit a **cubic Bezier curve** or **quadratic B-spline** to the sequence of centroids in the tracked history window.
  - Project the future path rollouts along this spline, incorporating both the estimated velocity direction and heading change rates, allowing Nav2 MPPI to plan smoother maneuvers around curved walking paths.

---

## [2026-06-04 13:30:00 -07:00] Iteration 4 Analysis

### 13. Dynamic Speed Scaling based on Pedestrian Proximity and Time-to-Collision (TTC)
* **Context:** In `ab_comparison_test.py` and `point_to_point_test.py`, the robot drives at a fixed maximum speed (`MAX_LINEAR_SPEED = 0.20` m/s) modulated only by the distance error to the target waypoint.
* **The Issue:** The control loop does not react directly to nearby moving obstacles. If a pedestrian steps into the path, the robot relies entirely on the local planner (Nav2 MPPI) to steer around them. In a narrow classroom environment, steering can fail or result in sudden, high-acceleration maneuvers.
* **Proposed Enhancement:**
  - Leverage the asynchronous WebSocket velocity estimates in `ab_comparison_test.py` to calculate the distance and estimated Time-to-Collision (TTC) for all tracked obstacles.
  - Dynamically scale the linear velocity commands. If a pedestrian is within a safety zone (e.g., $1.5\text{ m}$) or has a TTC below a threshold (e.g., $3.0\text{ s}$), decrease `MAX_LINEAR_SPEED` proportionally. This allows the robot to proactively slow down, giving the pedestrian time to pass and reducing the severity of required evasive steering.

### 14. Temporal Jitter Mitigation in A/B Run Logs via Synced Message Buffering
* **Context:** `ab_comparison_test.py` runs a control loop at $20\text{ Hz}$ and logs state variables at $10\text{ Hz}$ (`LOG_HZ`). Meanwhile, pedestrian velocity estimates are received asynchronously via WebSocket callbacks.
* **The Issue:** The log writes the robot pose and the most recently cached pedestrian state without temporal synchronization. This asynchronous cache lookup introduces temporal jitter (e.g., pairing a robot pose from time $t$ with an obstacle measurement from $t - 0.2\text{ s}$), which corrupts post-run comparison metrics.
* **Proposed Enhancement:**
  - Implement a synchronized message buffer in `ABComparisonTest`.
  - Capture precise Unix timestamps for both local `/odom` messages and incoming WebSocket `readout` frames.
  - When writing to the CSV, perform linear interpolation or nearest-timestamp alignment to pair each robot pose with the obstacle state matching that exact moment, eliminating log jitter.

### 15. Path Yielding and Recovery States for Blocked Trajectories
* **Context:** The state machines in `point_to_point_test.py` and `ab_comparison_test.py` are strictly sequential and command a minimum linear speed (`MIN_LINEAR_SPEED = 0.06` m/s) to overcome friction.
* **The Issue:** If a pedestrian stands directly in front of the robot, it will continue attempting to drive forward, causing it to push against the obstacle until the segment safety timeout ($20.0\text{ s}$) expires and shuts down the test.
* **Proposed Enhancement:**
  - Introduce an `OBSTACLE_PAUSE` state to the state machine in `ab_comparison_test.py`.
  - Transition to `OBSTACLE_PAUSE` if a tracked obstacle is detected directly ahead of the robot within a critical distance (e.g., $< 0.8\text{ m}$).
  - While paused, command zero velocity and temporarily suspend the segment timeout timer. Transition back to the active drive state once the path is clear for at least $1.5\text{ s}$, ensuring a safe recovery.

---

## [2026-06-04 16:30:00 -07:00] Iteration 5 Analysis

### 16. Bi-Directional Telemetry and Progress Reporting for A/B Runs
* **Context:** `server_x3.py` launches `ab_comparison_test.py` as a background process and broadcasts a basic status JSON (`{"status": "running"}`) to the Web GUI.
* **The Issue:** Once the process is active, the GUI has no visibility into the live progress of the test. The user cannot see the robot's current segment (e.g., driving forward, rotating, or returning), its real-time position, or progress metrics. The interface only knows that a test is active.
* **Proposed Enhancement:**
  - Update `ab_comparison_test.py` to send telemetry frames to `server_x3.py` at $5\text{ Hz}$ over the existing WebSocket connection using a new `"ab_test_telemetry"` message type.
  - The payload should include the active state (e.g., `"DRIVE_TO_B"`, `"ROTATE_180"`), target distance, distance traveled, and run elapsed time.
  - Modify `server_x3.py` to broadcast this telemetry to all connected clients, enabling the GUI to display a real-time progress bar and state indicators.

### 17. Subprocess Lifecycle Management and Orphan Prevention
* **Context:** `server_x3.py` starts test instances (`point_to_point_test.py` and `ab_comparison_test.py`) as background subprocesses.
* **The Issue:** The `cleanup()` function in `server_x3.py` terminates `_p2p_proc` but completely ignores `_ab_test_proc`. If the server exits, is stopped, or restarted, the running `ab_comparison_test.py` process becomes orphaned and will continue executing, potentially commanding `/cmd_vel` motor powers in the background, creating a physical safety hazard.
* **Proposed Enhancement:**
  - Update the `cleanup()` handler in `server_x3.py` to safely terminate and kill `_ab_test_proc`, matching the teardown pattern used for `_p2p_proc`.
  - Use process-group signaling (`os.killpg`) to ensure any nested child processes are cleanly reaped.
  - Implement a watchdog trigger to terminate active test runs if the client socket disconnects unexpectedly.

### 18. Cross-Track Error Correction Using Holonomic Strafing
* **Context:** The Yahboom X3 is a holonomic mecanum-wheel robot. However, the motion controllers in `ab_comparison_test.py` and `point_to_point_test.py` only command `linear.x` (forward speed) and `angular.z` (yaw correction).
* **The Issue:** Mecanum wheels are highly susceptible to lateral slip and drift. Without lateral correction, any sideways slip (e.g., from carpet fibers, unaligned rollers, or obstacles) goes uncorrected, leading the robot to drive parallel to the target vector but with a constant lateral offset (cross-track error) that prevents it from returning to the exact starting mark.
* **Proposed Enhancement:**
  - Calculate the cross-track error (the perpendicular distance of the robot's position from the straight line connecting Waypoint A and Waypoint B).
  - Implement a proportional (P) control loop for lateral error correction.
  - Feed this correction into `twist.linear.y` (sideways strafing speed) so the robot dynamically slides sideways to keep its center on the path vector, utilizing the platform's holonomic capabilities for precise path-following.

---

## [2026-06-04 19:30:00 -07:00] Iteration 6 Analysis

### 19. Depth-Aware 3D Centroid Reconstruction for Velocity Scaling
* **Context:** In `velocity_estimator.py`, the pedestrian centroid `(x_m, y_m)` in meters is calculated using a pinhole camera approximation: `x_m = (cx - w / 2.0) / fx`.
* **The Issue:** This formulation assumes a constant depth of $1.0\text{ m}$. In reality, the true lateral coordinate is $X = \frac{(u - c_x) \cdot Z}{f_x}$ where $Z$ is the distance to the obstacle. By ignoring $Z$, the estimated coordinates and displacements are scaled down by a factor of $Z$. A pedestrian walking at a distance of $3.0\text{ m}$ will have their position and displacement features underestimated by $300\%$, feeding distorted inputs to the MLP velocity predictor.
* **Proposed Enhancement:**
  - Retrieve the physical depth value $Z$ of the obstacle (e.g. by looking up the average depth in the centroid's region of interest).
  - Scale the horizontal coordinate proportionally: `x_m = (cx - w / 2.0) * Z / fx`.
  - This reconstructs the true 3D spatial coordinates in the robot's local frame, matching the absolute meter scale expected by the trained neural network.

### 20. Temporal Consistency and Interpolation in Track History
* **Context:** `ObstacleTracker` processes frames at $10\text{ Hz}$. If a pedestrian is occluded or a detection is missed for a frame, the track's age is incremented but no new entry is appended to the `history` queue.
* **The Issue:** The MLP feature extractor (`_build_window_features`) assumes that sequential elements in the `history` queue are exactly $0.1\text{ s}$ apart. If a detection was missed, the time delta between adjacent history elements is $0.2\text{ s}$ or more. Directly computing `dx = x - hist[i-1][0]` under these conditions falsely doubles the calculated displacement rate, introducing large noise spikes in the MLP inputs.
* **Proposed Enhancement:**
  - Track the frame timestamp or index for each history entry.
  - Detect gaps in track updates and insert interpolated centroid coordinates (e.g., via linear extrapolation or dead reckoning) to fill the missing steps.
  - This ensures a uniform temporal spacing of $0.1\text{ s}$ between all sequence frames, preserving the physical meaning of `dx` and `dy` features.

### 21. Direct Meters-Based Depth Processing to Avoid Normalization Artifacts
* **Context:** `ROS2Bridge._depth_cb` converts raw depth values (meters/millimeters) into a normalized, color-mapped BGR image to send to the GUI. The `VelocityEstimator` then takes this BGR image, converts it back to grayscale, and applies a static threshold of 120 to find obstacles.
* **The Issue:** The BGR image uses dynamic min-max normalization based on the closest and furthest points in the current frame. As the background changes (e.g., when the robot rotates), the normalization bounds shift, causing the grayscale intensity of static objects to fluctuate. This fluctuation leads to phantom detections or lost tracks. Furthermore, converting the raw depth to BGR and then back to grayscale adds unnecessary CPU overhead.
* **Proposed Enhancement:**
  - Expose the raw depth image array (in meters or millimeters) directly from the `ROS2Bridge` (e.g. `get_raw_depth_frame()`).
  - In `VelocityEstimator`, perform thresholding directly on the raw physical units: select pixels where `0.5m < depth < 4.0m`.
  - This makes obstacle detection completely invariant to background scaling, improves detection reliability, and reduces CPU utilization by eliminating the color-conversion pipeline.

---

## [2026-06-04 22:30:00 -07:00] Iteration 7 Analysis

### 22. Pedestrian Clearance Distance Logging for Quantitative Navigation Metrics
* **Context:** The EE244 project proposal lists "minimum clearance distance" as a key quantitative navigation metric to evaluate the safety of the A/B comparison runs.
* **The Issue:** The current logging schema in `ab_comparison_test.py` saves robot coordinates, segment identifiers, and maximum obstacle speed, but does not record the distance between the robot and nearby obstacles. Without this telemetry, it is impossible to evaluate whether the predictive controller (estimator ON) maintains a safer clearance from pedestrians than the reactive controller (estimator OFF).
* **Proposed Enhancement:**
  - Update the WebSocket readout payload parsed in `ab_comparison_test.py` to extract the spatial coordinates of each obstacle.
  - Calculate the real-time Euclidean distance to the nearest obstacle: $d_{min} = \min \sqrt{x_{obs}^2 + y_{obs}^2}$.
  - Log this distance at each interval in the CSV under the column `"min_obstacle_distance"`, enabling automated offline parsing of safety clearances.

### 23. Sensor Fusion using 2D LiDAR Point Cloud Clustering
* **Context:** The project proposal outlines a multi-modal estimator combining depth-image contours and LiDAR observations. Currently, `server_x3.py` passes the LiDAR object to `VelocityEstimator`, but the estimator does not utilize it.
* **The Issue:** The depth camera's narrow field of view ($\sim 60^\circ$) limits the tracking window; pedestrians walking to the side of the robot immediately disappear from the sequence history. Furthermore, depth cameras fail under bright lighting or complex shadows.
* **Proposed Enhancement:**
  - Subscribe to the `/scan` topic in `ROS2Bridge` and expose a `get_laser_points()` method.
  - In `VelocityEstimator`, apply a clustering algorithm (like DBSCAN) to the 2D laser points to extract obstacle centroids, and fuse them with the depth camera detections.
  - By utilizing the $360^\circ$ coverage of the LiDAR, the estimator can maintain track history even when pedestrians move out of the camera's front frustum.

### 24. SLAM-Based Odometry Drift Logging for Ground-Truth Evaluation
* **Context:** The closed-loop controllers in `ab_comparison_test.py` and `point_to_point_test.py` use the `/odom` topic for distance and rotation tracking.
* **The Issue:** Wheel slippage (especially during rapid $180^\circ$ spins) causes encoder-based odometry to drift. When the robot thinks it has returned to the start position $(0,0)$, it is physically offset from the tape mark. This drift goes unrecorded, preventing the comparison of how reactive vs. predictive steering affect wheel slip and localization quality.
* **Proposed Enhancement:**
  - Subscribe to the `/tf` topic in `ab_comparison_test.py` to monitor the `map -> odom` transform published by SLAM Toolbox.
  - At the end of the run, log the translation vector of this transform. This vector represents the exact coordinate drift corrected by the laser scan matcher.
  - Record the values in the CSV summary to quantitatively verify which driving mode causes less localization degradation.

---

## [2026-06-05 01:30:00 -07:00] Iteration 8 Analysis

### 25. Dynamic Costmap Overlay on Map Canvas in GUI
* **Context:** The Web GUI in `src/web/GUI.html` and `src/web/main.js` visualizes the occupancy grid map and robot pose, but does not display the obstacle costmap layers or the predictive trajectories.
* **The Issue:** While RViz2 can display costmap inflation, the user or audience viewing the web GUI during a live classroom demo cannot see the actual predictive cost boundaries or planner paths around the dynamic obstacle. This limits the visual impact of the live classroom demo.
* **Proposed Enhancement:**
  - Publish the predictive costmap updates via the WebSocket server (or a downsampled version of it).
  - Render the dynamic cost grid overlays directly on the GUI's map canvas, showing the safety zones expanding or shifting ahead of the pedestrian's path.

### 26. Heading Drift Correction via SLAM Loop Closures in Test Run Recovery
* **Context:** In `ab_comparison_test.py` and `point_to_point_test.py`, the state machine references starting coordinates at the beginning of each run to drive exactly 4.0m forward and back.
* **The Issue:** Over multiple consecutive A/B runs, the accumulation of wheel slippage and gyro drift leads to spatial drift relative to the physical tape mark. Since the script uses `/odom` directly (which drifts), the robot does not return to the exact physical starting point.
* **Proposed Enhancement:**
  - Retrieve the `map -> odom` transform from the SLAM Toolbox TF tree at the beginning of each segment.
  - Calculate starting and target coordinates in the `map` frame instead of the raw `odom` frame. This will align waypoints to the static environment map rather than drifting wheel odometry, ensuring that the robot returns to the exact tape mark on every single run.

---

## [2026-06-05 04:30:00 -07:00] Iteration 9 Analysis

### 27. Multi-Pedestrian Priority-Based Local Waypoint Halting
* **Context:** In `ab_comparison_test.py`, the robot drives at a fixed maximum speed scaled by the closest obstacle.
* **The Issue:** In crowded environments with multiple pedestrians crossing from different angles, a simple minimum-scaling across all obstacles can cause the robot to halt repeatedly or get stuck (the "freezing robot" problem) if any person is nearby, even if that person is walking away or parallel to the robot.
* **Proposed Enhancement:**
  - Implement a priority-based attention filter. Filter out obstacles whose heading angle is pointing away from the robot's future path frustum.
  - Only apply Proximity/TTC scaling for pedestrians whose predicted trajectories intersect the robot's linear path vector, allowing the robot to maintain speed when pedestrians are walking parallel to or away from it.

### 28. Dynamic Look-Ahead Distance for Heading Corrections
* **Context:** In `ab_comparison_test.py` and `point_to_point_test.py`, the heading correction during forward drive uses a fixed gain `KP_YAW` and a constant reference `target_yaw`.
* **The Issue:** If the robot drifts laterally (e.g. slips sideways on carpet), it corrects its heading to face parallel to the target vector, but it remains laterally offset (cross-track error) because it only controls heading, not position.
* **Proposed Enhancement:**
  - Calculate the lateral cross-track error.
  - Dynamically adjust the `target_yaw` to steer towards a look-ahead point on the path vector (similar to a Pure Pursuit controller) rather than keeping it constant. This will guide the robot back onto the straight path line rather than driving parallel to it with an offset.

---

## [2026-06-05 07:30:00 -07:00] Iteration 10 Analysis

### 29. Adaptive Lateral Bypass Direction Selection
* **Context:** In `ab_comparison_test.py`'s `_update_bypass_offset()`, when an obstacle blocks the path, the robot decides to steer right (offset -0.7m) or left (offset +0.7m) depending solely on whether the obstacle's centroid is slightly to the left or right of the center.
* **The Issue:** The controller assumes the target bypass side is physically clear. In a real classroom, corridor, or laboratory, one side is often blocked by a wall, desk, or another pedestrian. Strafing into a blocked sector results in a physical collision or planner failure.
* **Proposed Enhancement:**
  - Utilize the LiDAR scan ranges to scan the left and right sectors ($\pm 45^\circ$ to $\pm 135^\circ$ relative to the robot's heading) to check for clearance.
  - Choose the bypass direction (left vs. right) that has the largest free distance gap in the LiDAR sectors, preventing the robot from strafing directly into walls or static furniture.

### 30. Smoothed Lateral Velocity Profiles for Mecanum Slip Prevention
* **Context:** The lateral command velocity `vy_cmd` is calculated using a proportional gain: `vy_cmd = (self.target_lateral_offset - path_y) * KP_LATERAL` limited to `MAX_LATERAL_SPEED` (0.15 m/s).
* **The Issue:** Setting a step-change target lateral offset of $\pm 0.7\text{ m}$ instantly commands a maximum lateral velocity spike (0.15 m/s) to the mecanum wheels. This sudden high lateral acceleration causes wheel slippage, which corrupts encoder-based odometry and introduces dead-reckoning errors.
* **Proposed Enhancement:**
  - Apply a linear ramp or a trapezoidal velocity profiler (rate-limiter) to `vy_cmd` to limit the lateral acceleration.
  - Slowly increase the lateral command velocity, preserving wheel traction and maintaining accurate EKF odometry updates during sudden sideways evasive maneuvers.

---

## [2026-06-05 10:45:00 -07:00] Iteration 11 Analysis

### 31. Dynamic Path Re-planning Timeout Scaling
* **Context:** When an obstacle blocks the path in `ab_comparison_test.py`, the robot sets `self.is_paused = True` and waits while it shifts laterally. To ensure the robot doesn't trigger a safety timeout while waiting or bypass-strafing, the state segment timer is paused during this time.
* **The Issue:** If a pedestrian stands directly in front of the robot and follows its movements, or if the path is permanently blocked, the robot will remain paused indefinitely. Without a fallback timeout for the blocked state, the robot can get trapped in a perpetual block-waiting state.
* **Proposed Enhancement:**
  - Implement a blocked-duration threshold (e.g., `MAX_BLOCKED_TIME = 8.0` seconds).
  - If the robot remains blocked or paused for longer than this threshold, it should transition to an active recovery/warning state (e.g., reversing slightly or sounding a buzzer alert) to prompt the pedestrian to move, rather than staying stuck indefinitely.

### 32. Damping Gain Scheduling for Lateral Alignment
* **Context:** The lateral speed controller commands velocity based on lateral offset error: `vy_cmd = (self.target_lateral_offset - path_y) * KP_LATERAL`.
* **The Issue:** Mecanum wheels experience high lateral slip. When command step-inputs (like shifting $0.7\text{ m}$) are applied, the proportional-only lateral controller is prone to overshooting the target lateral offset and oscillating, which can cause the robot to drift too far to the side and hit obstacles.
* **Proposed Enhancement:**
  - Implement a lateral Proportional-Derivative (PD) controller or gain scheduler for the lateral command velocity `vy_cmd`.
  - Calculate the lateral error rate-of-change (cross-track velocity) and apply a derivative damping term to slow down the lateral velocity smoothly as the robot approaches the target lateral offset, preventing overshoot.

---

## [2026-06-05 19:20:00 -07:00] Iteration 12 Analysis

### 33. Dynamic Temporal Scaling of Centroid Displacements
* **Context:** The MLP is trained on constant-frequency displacement vectors (e.g., at exactly 10Hz/0.1s delta). Under high CPU/GPU load on the Jetson Orin, the `_inference_loop` can lag, meaning the actual time delta $dt$ is greater than 0.1s.
* **The Issue:** Directly calculating $dx = x_t - x_{t-1}$ under varying $dt$ distorts the model inputs, introducing speed estimation errors because the displacement over a larger time gap is incorrectly treated as occurring in 0.1s.
* **Proposed Enhancement:**
  - Measure the actual elapsed time delta $dt_{actual}$ since the last update for each tracked obstacle.
  - Scale the computed displacements to the standard training time interval: $dx_{scaled} = dx \cdot \frac{0.1}{dt_{actual}}$ and $dy_{scaled} = dy \cdot \frac{0.1}{dt_{actual}}$. This stabilizes features against thread schedule latency and improves velocity estimation accuracy.

### 34. Distance-Adaptive Area Thresholding for Near-Field Detections
* **Context:** `velocity_estimator.py` uses a hardcoded `MIN_BLOB_AREA = 500` pixels for contour detection.
* **The Issue:** Farther obstacles are small and may fall below 500 pixels (causing track loss), whereas closer obstacles in tight areas occupy a large pixel area but can be split or truncated near boundaries, making a static area threshold problematic.
* **Proposed Enhancement:**
  - Make the contour area threshold dynamic based on depth range, e.g., $\text{MIN\_BLOB\_AREA}(Z) = \text{clamp}(K / Z^2, 150, 1000)$.
  - Lower the lower bound of depth thresholding to $0.25\text{m}$ (matching the physical Astra Pro limit) to allow the robot to maintain feature detection and navigate extremely close obstacle fields in narrow corridors.

### 35. Continuous, Gap-Based Lateral Bypass Offset Calculation
* **Context:** In `ab_comparison_test.py`, the bypass offset is a fixed, step-input value of $\pm 0.25\text{m}$ or $\pm 0.45\text{m}$ depending on obstacle centers.
* **The Issue:** Commanding a fixed bypass offset can force the robot directly into walls or static furniture in narrow spaces if the gap between the obstacle and the wall is smaller than the commanded offset.
* **Proposed Enhancement:**
  - Use LiDAR scan ranges in the left/right bypass zones to continuously compute the available free gap.
  - Calculate a dynamic bypass offset that centers the robot in the largest clear corridor, slowing down or halting if no safe gap exists, rather than committing to binary left/right step-offsets.

### 36. Cropped Contour Masking for Centroid Extraction
* **Context:** In `_extract_depth_centroids`, to query raw depth values inside a contour, the code currently instantiates a full-sized mask (`np.zeros_like(mask)` of size 640x480) and draws a contour on it, then indexes `raw_depth_frame`.
* **The Issue:** Creating and processing full-sized image masks for multiple contours at 10Hz consumes unnecessary CPU cycles and memory bandwidth on the Jetson Orin.
* **Proposed Enhancement:**
  - Crop the bounding box of the contour: `x, y, w, h = cv2.boundingRect(cnt)`.
  - Allocate a small local mask of size $w \times h$ and draw the translated contour onto it.
  - Index the cropped slice of `raw_depth_frame[y:y+h, x:x+w]` using the local mask. This reduces memory allocations and pixel comparison cycles by over 95%.

### 37. Native Raw Depth Retrieval for AstraCamera
* **Context:** The physical `AstraCamera` class in `drivers_x3.py` does not implement `get_raw_depth_frame()`, forcing the estimator to fall back to colorised BGR thresholding logic on hardware, whereas `ROS2Bridge` supports raw depth.
* **The Issue:** Falling back to color-mapped BGR parsing adds CPU overhead (conversion and normalization) and makes the thresholding sensitive to dynamic min-max normalization bounds.
* **Proposed Enhancement:**
  - Implement `get_raw_depth_frame()` inside `AstraCamera` in `src/drivers_x3.py` by retrieving the raw OpenNI2 uint16 buffer, converting to meters, and flipping.
  - This allows the estimator to use direct physical meter thresholding universally, improving reliability and efficiency on the physical robot.

---

## [2026-06-05 20:00:00 -07:00] Iteration 13 Analysis

### 38. Velocity-Projected Constant Velocity Tracker Association
* **Context:** The `ObstacleTracker` matches new centroids to existing tracks using the Euclidean distance from the track's previous raw position.
* **The Issue:** For fast-moving pedestrians or under reduced frame rates (e.g. from 10Hz to 5Hz), the distance a pedestrian moves between frames can exceed the match threshold, leading to tracking loss, or it can be closer to a different pedestrian's track, leading to identity switches.
* **Proposed Enhancement:**
  - Leverage the estimated velocity $(v_x, v_y)$ of the track from the previous frame to project the track's expected position: $x_{pred} = x_{prev} + v_x \cdot dt$ and $y_{pred} = y_{prev} + v_y \cdot dt$.
  - Calculate the nearest-centroid matching distance relative to the projected target position $(x_{pred}, y_{pred})$ instead of the previous raw centroid $(x_{prev}, y_{prev})$.
  - This stabilizes associations under high-velocity movement or low frame rates without needing a complex Kalman Filter.

### 39. Map-Masked LiDAR Front Blockage Filtering
* **Context:** In `ab_comparison_test.py`, the front path is declared blocked if any LiDAR point falls within a forward sector bounding box ($x < 0.75$m, $|y| < 0.4$m).
* **The Issue:** In tight corridors or narrow doorways, minor yaw drift or rotation places static walls or door frames inside the forward sector, causing the robot to falsely trigger a blockage pause and eventually timeout.
* **Proposed Enhancement:**
  - Retrieve the static occupancy grid map from SLAM Toolbox via `ros_bridge.get_occupancy_grid()`.
  - Transform front-sector LiDAR detections into the global map frame using the current robot pose.
  - Check the map cell occupancy value at those coordinates. If the cell is classified as a known static obstacle (wall), ignore the point for path blockage checks. Only trigger path pause if the blocking point falls into known free space, isolating dynamic pedestrians from static walls.

### 40. Swept-Volume Rotational Safety Fields
* **Context:** During turnaround states (`ROTATE_180` and `ROTATE_HOME`), the robot executes in-place rotations, controlling only angular velocity.
* **The Issue:** The robot's rectangular corners sweep out a larger radius (swept-volume) than its stationary width. If a turn is initiated close to a table leg or wall, its corners can collide with obstacles during the spin.
* **Proposed Enhancement:**
  - Before and during rotation states, scan the $360^\circ$ LiDAR profile.
  - If any obstacle point is within the robot's circumscribed diagonal radius (plus a safety buffer, e.g. 0.38m), compute the direction of the obstacle.
  - Command a minor lateral/linear translation (using the mecanum wheels' holonomic capability) to drift the robot away from the obstacle while it spins, or abort the rotation and back up before executing the turnaround.

### 41. Vectorized Feature Construction in Track History Processing
* **Context:** In `_build_window_features`, the history queue is converted element-by-element using Python list insertions and loops to flatten coordinate displacement sequences into a $(1, 40)$ feature vector.
* **The Issue:** Doing this element-by-element looping for every track (up to 5 obstacles) at 10Hz introduces Python interpreter overhead and unnecessary memory allocations.
* **Proposed Enhancement:**
  - Convert the track history deque directly to a numpy array of shape $(T, 3)$ representing the coordinate history.
  - Extract the $r_x$ (depth) and $r_y$ (negated camera X) arrays.
  - Use `np.diff` with `prepend` to compute displacements $dx$ and $dy$ instantly: `dx = np.diff(rx, prepend=rx[0])` and `dy = np.diff(ry, prepend=ry[0])`.
  - Stack and flatten the features using `np.column_stack` or `np.concatenate` to return the $(1, 40)$ shape. This vectorization reduces the computation time of feature prep to negligible levels.

### 42. Shell Sourcing Elimination for ROS2 Subprocesses
* **Context:** `server_x3.py` launches ROS2 nodes (Gazebo, SLAM, hardware bringup) using a nested bash shell `bash -c "source /opt/ros/... && source ... && ros2 launch ..."` via `subprocess.Popen`.
* **The Issue:** Spawning a subshell and sourcing heavy setup bash scripts consumes significant CPU cycles (100% core spikes for 1-2 seconds) and delays startup sequences.
* **Proposed Enhancement:**
  - Since the parent `server_x3.py` is already launched in an environment with ROS2 and workspace setup sourced, the environment variables (`PATH`, `PYTHONPATH`, etc.) are already populated.
  - Execute `ros2` directly as an executable array: `['ros2', 'launch', 'yahboomcar_nav', launch_file]` using `subprocess.Popen`.
  - Pass the current `os.environ` or a cleaned copy of it. This removes shell startup overhead and prevents startup CPU spikes on the Jetson Orin.

---

## [2026-06-05 21:00:00 -07:00] Iteration 14 Analysis

### 43. Temporal Depth Filtering for Centroid Smoothing
* **Context:** In `velocity_estimator.py`, the depth coordinate $Z$ of each centroid is calculated per frame using the median depth of the contour pixels.
* **The Issue:** Shape fluctuations of the contour (e.g. from swinging arms or shadows) cause high-frequency depth measurement jitter. This jitter directly impacts the coordinate displacement values ($dx, dy$) in the MLP feature vector, reducing velocity prediction accuracy.
* **Proposed Enhancement:**
  - Implement a low-pass Exponential Moving Average (EMA) filter on the tracked depth coordinate $Z$ at the tracker level: $Z_{filtered} = \alpha \cdot Z_{new} + (1 - \alpha) \cdot Z_{prev}$ (e.g. with $\alpha = 0.7$).
  - Smoothing the depth sequence stabilizes the displacement features fed to the MLP, reducing output velocity variance and improving overall estimation accuracy.

### 44. Vertical Centroid Fusion for Partially Occluded Pedestrians
* **Context:** In `velocity_estimator.py`, depth contours are extracted using `cv2.findContours` with the `RETR_EXTERNAL` mode.
* **The Issue:** In tight environments with furniture (chairs, desks), a pedestrian's body can be divided into separate visual regions (e.g., legs and torso). This leads to multiple separate tracks for a single person, splitting the features and corrupting centroid displacement metrics.
* **Proposed Enhancement:**
  - Implement a vertical fusion/grouping pass after extracting centroids.
  - If two distinct contours lie at similar depth ranges ($|Z_1 - Z_2| < 0.15$m) and have overlapping horizontal bounds ($|\Delta x| < 0.25$m), merge them into a single tracked obstacle.
  - This preserves tracking continuity and feature integrity for partially occluded humans in tight classroom layouts.

### 45. Blended Diagonal Bypass Path Profiling
* **Context:** In `ab_comparison_test.py`, the robot halts completely (`speed = 0.0`) while it strafes laterally to line up with the target bypass offset.
* **The Issue:** Halting forward motion during strafing wastes momentum, increases travel time, and leads to blocky, non-fluid movements.
* **Proposed Enhancement:**
  - Blend forward speed (`linear.x`) and lateral speed (`linear.y`) dynamically to profile a coordinated diagonal bypass path.
  - Scale the forward velocity command proportionally to the lateral error (e.g., $v_x = v_{max} \cdot (1 - \text{clamp}(|path\_y - offset| / 0.3, 0, 1))$) so that the robot naturally slows down slightly and glides diagonally around the obstacle rather than coming to a dead stop.

### 46. Batched MLP Inference for Multi-Track Scaling
* **Context:** In `velocity_estimator.py`, neural network inference is run inside a Python loop sample-by-sample for each active track using `self._model(x_tensor)`.
* **The Issue:** Sequential model evaluations on individual tracks create high overhead from PyTorch-to-Python bindings and execution launches, which wastes CPU cycles on the Jetson.
* **Proposed Enhancement:**
  - Collect features for all active, eligible tracks into a single unified NumPy array of shape $(N, 40)$.
  - Convert this batch array into a single PyTorch tensor `x_batch` and perform a single batched inference call: `preds_batch = self._model(x_batch).numpy()`.
  - Distribute the resulting predictions back to their corresponding tracks. This reduces the inference launch overhead to a constant time factor, dramatically improving scalability.

### 47. Event-Driven or Throttled Bypass Optimization
* **Context:** In `ab_comparison_test.py`'s control loop running at 20Hz, `_update_bypass_offset` parses the entire LiDAR scan and camera estimates on every iteration.
* **The Issue:** The LiDAR scan publishes at ~8Hz and camera estimations run at 10Hz. Running the bypass check at 20Hz results in redundant computations on identical data, wasting CPU bandwidth.
* **Proposed Enhancement:**
  - Implement a dirty flag or rate-limiter that restricts `_update_bypass_offset` executions to 10Hz or only immediately after receiving a new WebSocket readout or LiDAR scan.
  - This halves the CPU usage of the reactive avoidance parser in the test script.

---

## [2026-06-05 22:00:00 -07:00] Iteration 15 Analysis

### 48. Local Trajectory Translation Normalization
* **Context:** The feature vector built in `_build_window_features` feeds absolute coordinates $rx, ry$ (relative to the robot) along with displacements $dx, dy$ into the MLP model.
* **The Issue:** Feeding absolute relative coordinates causes the model to overfit to the spatial coordinates of training samples (e.g. predicting differently based on absolute distance). It makes predictions sensitive to the starting point of the pedestrian in the camera frame.
* **Proposed Enhancement:**
  - Record the starting relative coordinates $(rx_0, ry_0)$ of the track at the beginning of the active history window.
  - For all elements in the window, subtract this start offset: $rx_{norm} = rx - rx_0$ and $ry_{norm} = ry - ry_0$.
  - Feed $[rx_{norm}, ry_{norm}, dx, dy]$ into the model. This makes the spatial trajectory input translation-invariant, forcing the MLP to generalize based purely on trajectory shape.

### 49. LiDAR-Camera Joint Frustum Association and FOV Handover
* **Context:** The camera's horizontal FOV is limited ($\approx 60^\circ$), which prevents tracking pedestrians who move to the sides of the robot in narrow areas.
* **The Issue:** Pedestrians moving laterally close to the robot disappear from the camera, causing the tracking sequence to break even if they are still detected by the $360^\circ$ LiDAR scan.
* **Proposed Enhancement:**
  - Project the 3D centroids of dynamic LiDAR clusters into the camera frame using camera intrinsic calibration parameters.
  - If the projected coordinates lie inside the camera FOV, associate them with the camera's depth centroids (multi-modal fusion).
  - If the cluster moves outside the camera's frustum limits, smoothly transition the track to run solely on LiDAR centroids. This ensures seamless coverage expansion and tracking continuity in tight quarters.

### 50. Footprint-Swept Corridor Expansion for Holonomic Path Protection
* **Context:** In `ab_comparison_test.py`, the linear speed scaling and TTC calculations use a static lateral corridor width (`LATERAL_THRESHOLD = 0.35`m) to look for blocking obstacles in front of the robot.
* **The Issue:** When the robot strafes laterally (holonomic bypass), its physical trajectory is diagonal, not straight. A static lateral corridor centered on the robot's heading will fail to detect obstacles in its sideways path of travel, risking a side collision.
* **Proposed Enhancement:**
  - Expand the lateral safety corridor dynamically during bypass maneuvers to include the target lateral offset and current lateral speed: $\text{LATERAL\_THRESHOLD} = 0.35 + |path\_y| + |vy\_cmd| \cdot \text{lookahead\_time}$.
  - This ensures the proximity and collision estimators cover the entire swept volume of the diagonal trajectory.

### 51. Client-Side Drawing Overlay Delegation
* **Context:** When object detection is active, the WebSocket server (`server_x3.py`) draws bounding box rectangles and labels on the image frame on the Jetson CPU and encodes it as a separate JPEG.
* **The Issue:** Drawing on the image requires copying the entire 640x480 frame and performing a second JPEG compression, wasting significant CPU cycles and increasing network packet size.
* **Proposed Enhancement:**
  - Send only the raw, un-annotated JPEG camera frame over WebSockets.
  - Rely on the Web GUI browser to draw the bounding boxes and text labels dynamically on its HTML5 canvas using the `"detections"` data array already included in the JSON readout payload. This removes rendering overhead and redundant JPEG compression from the Jetson CPU.

### 52. Decimated Depth Arrays for Centroid Median Calculation
* **Context:** In `velocity_estimator.py`'s centroid extraction, `np.median` is run on the filtered depth values inside each contour to calculate distance $Z$.
* **The Issue:** Large obstacle contours (e.g., when close to the camera in tight areas) can contain thousands of pixels, making array filtering and sorting for the median computationally expensive.
* **Proposed Enhancement:**
  - Decimate the contour depth pixel array before computing the median, e.g., if `len(depth_vals) > 200`, slice it as `depth_vals[::len(depth_vals) // 200]`.
  - Taking a uniform sample of 200 pixels yields a statistically identical median (within sub-millimeter precision) but reduces median sorting time by over 90% for large objects.

---

## [2026-06-05 23:00:00 -07:00] Iteration 16 Analysis

### 53. Bipartite Matching for Tracker Centroid Association
* **Context:** Detections are matched to active tracks in `ObstacleTracker` using a greedy sequential nearest-centroid loop.
* **The Issue:** The greedy approach is highly order-dependent. If two pedestrians walk in close proximity, a track processed early can "steal" a centroid that belongs to a different track, causing tracking loss and identity switches.
* **Proposed Enhancement:**
  - Construct a full cost matrix representing distance offsets between all active tracks and new centroids.
  - Solve the association problem globally using a bipartite matching solver or a sorted-greedy minimum cost matching algorithm.
  - Global matching ensures optimal overall pairing, stabilizing track history sequences and reducing MLP input noise.

### 54. Morphological Flood-Fill Hole Correction for Specular Dropouts
* **Context:** In `_extract_depth_centroids`, binary masks are created by range-checking depth values and applying morphological opening/closing.
* **The Issue:** Pedestrians with highly reflective clothing or badges create specular reflections, resulting in invalid (zero/NaN) depth readings. This divides a single human contour into multiple smaller fragments, dropping their area below `MIN_BLOB_AREA` and causing detection failure.
* **Proposed Enhancement:**
  - Apply a binary flood-fill hole correction pass to the thresholded mask before extracting contours.
  - Flood-fill from the image borders, invert the result, and OR it with the original mask. This seals all internal dropouts within the pedestrian's body, maintaining a solid unified contour and ensuring reliable area thresholding.

### 55. Predictive Vector-Field Orientation Alignment
* **Context:** During driving states in `ab_comparison_test.py`, the robot heading is locked to a static yaw angle parallel to the waypoints segment.
* **The Issue:** In tight hallways or angled spaces, minor lateral drift forces the robot to maintain a heading that is not parallel to the actual walls, leading to uneven clearances and potential corner collisions.
* **Proposed Enhancement:**
  - Retrieve the local free space vector field from the SLAM Toolbox grid.
  - Dynamically align the robot's target heading parallel to the local centerline corridor tangent instead of a static yaw coordinate.
  - This keeps the robot chassis aligned parallel to the surrounding walls, maximizing side clearances during forward and lateral motions.

### 56. SIMD-Optimized OpenCV inRange Thresholding
* **Context:** In `velocity_estimator.py`, depth thresholding is performed using multi-stage NumPy boolean comparisons (`(raw_depth_frame >= 0.5) & ...`).
* **The Issue:** Evaluating multiple boolean matrices on the CPU for all 307,200 pixels in Python is slow and consumes memory bandwidth.
* **Proposed Enhancement:**
  - Replace NumPy comparison blocks with OpenCV's C++ native `cv2.inRange()` function after a quick `np.nan_to_num()` cleaning pass.

### 57. Binary Serialization of Depth Frames via WebSockets
* **Context:** When depth visualization is enabled, `server_x3.py` base64-encodes the compressed JPEG depth frame and sends it inside the JSON readout text payload.
* **The Issue:** Base64 encoding adds a 33% size overhead and consumes CPU time for text serialization and deserialization, bloating WebSocket traffic.
* **Proposed Enhancement:**
  - Exclude the `depth_image` string from the JSON readout text frame.
  - Compress the depth frame and broadcast the raw JPEG bytes directly as a binary WebSocket frame, tagged with a unique header (e.g. `b'DEPT'`).
  - This eliminates base64 encoding and decoding overhead, cutting transmission latency and bandwidth by 33%.

---

## [2026-06-06 00:00:00 -07:00] Iteration 17 Analysis

### 58. Kinematic Derivative Enrichment for MLP Input Vectors
* **Context:** Features mapped to the MLP model consist of raw positions $rx, ry$ and spatial displacements $dx, dy$ over a 10-frame window.
* **The Issue:** A flat position/displacement history vector provides limited temporal context of movement dynamics. The model lacks direct, explicit inputs for velocity, acceleration, and angular rates, making predictions under nonlinear walking profiles less accurate.
* **Proposed Enhancement:**
  - Enrich the feature vector with higher-order kinematic derivatives calculated over the history queue.
  - Append computed velocities ($v_x = dx/dt$), accelerations ($a_x = (v_x - v_{x,prev})/dt$), heading angles ($\theta = \text{atan2}(dy, dx)$), and angular rates ($\omega = d\theta/dt$) for all points in the window.
  - Providing these explicit physical features to the model structure dramatically reduces training regression errors and improves live velocity prediction.

### 59. Focal-Length Bounding Box Projection Fallback for Blind Spots
* **Context:** In `velocity_estimator.py`, pedestrian centroids are calculated by thresholding the depth frame.
* **The Issue:** If a pedestrian walks extremely close to the camera (e.g., $<0.6$m), the camera enters its physical depth blind spot, producing zero-depth readings. This fragments the binary mask and breaks centroid tracking exactly when the person is nearest to the robot.
* **Proposed Enhancement:**
  - Implement a bounding box tracker on the RGB or IR camera stream as a fallback.
  - When the depth centroid fails due to extreme proximity, track the person's bounding box and project their distance $Z$ using a pinhole focal-length scaling model: $Z_{est} = (W_{human} \cdot f_x) / w_{pixels}$.
  - This allows the robot to maintain feature tracking and obstacle awareness inside the camera's physical depth blind spot.

### 60. Global SLAM-Path Waypoint Interpolation Recovery
* **Context:** In `ab_comparison_test.py`, the robot drives a straight line between Waypoints A and B, using a 1D lateral offset to bypass obstacles.
* **The Issue:** If a permanent wall, furniture, or group of pedestrians completely blocks the straight-line corridor beyond the bypass offset limits, the robot remains stuck or hits a timeout.
* **Proposed Enhancement:**
  - Implement a recovery state that queries the SLAM path planner (Nav2) to compute a global detour route around the blockage.
  - Subdivide the detoured SLAM path into short straight-line virtual sub-waypoints and execute the local state machine along this detour.
  - This combines the direct speed-modulated evaluation with global navigation capabilities to resolve otherwise fatal hallway blockages.

### 61. Map Hash Checksumming and Delta Compression
* **Context:** The WebSocket server pushes the entire SLAM occupancy grid map to clients every 1.0 second in `map_push_loop`.
* **The Issue:** The map array is large (often multiple megabytes), and compressing/broadcasting it repeatedly consumes high CPU time and network bandwidth, even when the robot is stationary and the map is unchanged.
* **Proposed Enhancement:**
  - Compute a hash checksum (e.g. MD5 or SHA-256) of the occupancy grid data on each update.
  - Skip encoding and transmission if the checksum matches the previously sent map.
  - Additionally, implement delta encoding to only compress and transmit cells within the bounding box of modified map sectors. This cuts map transmission bandwidth by over 95%.

### 62. Pre-Allocated Static Tracks and Numpy Deque Buffers
* **Context:** `ObstacleTracker` dynamically instantiates track dictionaries and `deque` queues, deleting them when aged out.
* **The Issue:** Continuously allocating and deallocating memory objects at 10Hz causes heap memory fragmentation in Python and triggers frequent garbage collection cycles, increasing CPU latency jitter.
* **Proposed Enhancement:**
  - Pre-allocate a static list of trackers up to `MAX_OBSTACLES = 5` at initialization.
  - Replace the Python `deque` history queue in each track with a static pre-allocated NumPy array buffer.
  - Update tracks by writing to slices of the pre-allocated buffers and toggle an active/inactive boolean flag. This eliminates runtime allocations and garbage collection interruptions in the tracking pipeline.

---

## [2026-06-06 01:00:00 -07:00] Iteration 18 Analysis

### 63. Robust Coordinate Clamping and Velocity Bound Constraints
* **Context:** Sequence displacement coordinates ($dx, dy$) derived from raw centroids are directly scaled and fed into the MLP velocity predictor.
* **The Issue:** Occasional centroid tracking glitches (e.g. contour splitting or edge anomalies) create unphysical coordinate jumps. These out-of-distribution values map to extreme values through feature scaling, leading the MLP to predict wild velocity spikes.
* **Proposed Enhancement:**
  - Implement a clamping/clipping logic on input displacements before scaling: clamp $|dx|$ and $|dy|$ to a maximum of $0.25$m per frame (equivalent to a maximum pedestrian speed of 2.5 m/s).
  - This bounds the feature space, preventing isolated tracking noise from corrupting regression inference.

### 64. Depth-Gradient Edge Segmentation for Overlapping Obstacles
* **Context:** In `velocity_estimator.py`, contours are extracted from the range-thresholded binary depth mask.
* **The Issue:** When a pedestrian stands close to static objects (desks, chair backs) or other people, their depth ranges can overlap, merging them into a single massive contour. This shifts the tracking centroid away from the true human center and dampens estimated velocity.
* **Proposed Enhancement:**
  - Calculate spatial depth gradients ($dZ/dx, dZ/dy$) inside large candidate contours.
  - Locate gradient spikes, which mark physical depth boundaries between overlapping objects (e.g. person at 1.2m vs table leg at 0.8m).
  - Apply a watershed or depth-split segmentation along these gradient boundaries to partition the merged contour into distinct objects before centroid evaluation.

### 65. Forward-Projected Path Collision Corridors
* **Context:** In `ab_comparison_test.py`, the bypass side check evaluates side clearances inside static circular LiDAR sectors ($\pm 30^\circ$ to $\pm 135^\circ$).
* **The Issue:** If the robot is in a narrow hallway, static obstacles or walls that are behind the robot's front wheel line will intersect the wide circular sectors and trigger false "blocked" bypass flags, freezing navigation.
* **Proposed Enhancement:**
  - Define side safety zones as rectangular corridors projected along the active diagonal bypass paths.
  - Filter LiDAR points to check for collisions only within these forward-directed bounding boxes.
  - This ignores static objects behind or far lateral to the robot's projected trajectory, maximizing traversal efficiency in tight hallways.

### 66. ONNX Runtime Conversion for PyTorch-Free Execution
* **Context:** `velocity_estimator.py` imports `torch` to load the TorchScript JIT MLP and perform forward pass inference.
* **The Issue:** PyTorch consumes over 120MB of RAM and takes 2-3 seconds to load on startup. It has high binding overhead when evaluating small $(1, 40)$ matrices.
* **Proposed Enhancement:**
  - Export the TorchScript MLP to ONNX format.
  - Replace PyTorch imports and tensor operations with `onnxruntime` and standard NumPy arrays in `velocity_estimator.py`.
  - ONNX Runtime has a negligible memory footprint, loads instantly, and runs optimized C++ execution kernels on the Jetson CPU, freeing up substantial RAM and CPU resources.

### 67. Battery Voltage Query Throttling in ROS2 Bridge
* **Context:** In the 20Hz `broadcast_loop` of `server_x3.py`, `drive.get_battery_voltage()` is called on every iteration to fetch the battery voltage from the ROS2 `/voltage` topic subscriber.
* **The Issue:** Querying voltage at 20Hz causes unnecessary mutex lock contention and CPU overhead in the ROS2 bridge for a value that changes very slowly (over minutes).
* **Proposed Enhancement:**
  - Cache the battery voltage value within the bridge or the broadcast loop.
  - Only query `drive.get_battery_voltage()` once per second (1Hz). This removes 95% of lock accesses and topic reads, reducing bridge execution cost.

---

## [2026-06-06 02:00:00 -07:00] Iteration 19 Analysis

### 68. Kinematic Back-Propagation for Track Initialization Padding
* **Context:** Newly created tracks are padded in `_build_window_features` by duplicating the first observed centroid coordinate, resulting in zero displacements ($dx, dy = 0.0$) for the padded historical entries.
* **The Issue:** Forcing displacements to zero at track initialization confuses the MLP (trained on true moving sequences), leading to highly inaccurate velocity predictions during the first second (10 frames) of tracking.
* **Proposed Enhancement:**
  - Calculate the initial movement direction and speed over the first 2-3 frames of a new track.
  - Back-propagate (dead reckon) the coordinates backward in time to pad the history window.
  - If a new track begins with positions $c_0, c_1$, set historical padded entries to $c_{padded}(i) = c_0 - (0 - i) \cdot (c_1 - c_0)$. This populates the window with a realistic motion vector, eliminating the 1-second prediction lag.

### 69. Multi-Sensor Boundary Contrast Cross-Checking
* **Context:** In tight classroom layouts, a pedestrian standing close to a wall creates a single continuous depth contour, merging the human into the static wall structure.
* **The Issue:** The merged contour shifts the centroid away from the true human center, and the static wall component dampens the estimated displacement, preventing the estimator from tracking the pedestrian's movement.
* **Proposed Enhancement:**
  - Map depth contour boundaries to corresponding angles in the high-resolution 2D LiDAR scan.
  - Check the LiDAR range profile along this boundary: walls appear as continuous straight lines, while pedestrians produce a distinct, slightly curved shape.
  - Use the LiDAR-detected boundary to crop and isolate the dynamic pedestrian pixels in the depth frame, segmenting the merged contour.

### 70. Omnidirectional Direction-of-Travel Coordinate Projection
* **Context:** Proximity and Time-to-Collision (TTC) speed scaling calculations in `ab_comparison_test.py` assume forward travel and only evaluate obstacles in the forward sector of the robot's body frame.
* **The Issue:** During lateral strafing maneuvers (holonomic bypass), the robot moves sideways or diagonally. Obstacles situated in its path of travel (but to the side of its body) are ignored by the forward-looking scaling logic, creating a collision risk.
* **Proposed Enhancement:**
  - Transform all tracked pedestrian coordinates and velocities into a coordinate frame aligned with the robot's commanded velocity vector: $V_{robot} = [vx_{cmd}, vy_{cmd}, 0]^T$.
  - Run all proximity and TTC scaling checks in this direction-of-travel frame. This ensures the speed controller naturally reacts to obstacles situated in its actual path of travel, providing omnidirectional safety.

### 71. Nav2 Action Status Query Throttling
* **Context:** In `broadcast_loop` of `server_x3.py`, `nav2_client.get_status()` is invoked at 20Hz to populate the JSON readout telemetry packet.
* **The Issue:** Fetching status from Nav2's action client requires querying action servers and handling future objects, which creates lock overhead in ROS2. Running this at 20Hz is unnecessary since navigation states change slowly.
* **Proposed Enhancement:**
  - Cache the Nav2 status dictionary inside the broadcast loop.
  - Throttle calls to `nav2_client.get_status()` to 2Hz (every 10 iterations). This reduces Nav2 status retrieval overhead by 90%.

### 72. Pure NumPy Feature Normalization (Eliminating Scikit-Learn)
* **Context:** `velocity_estimator.py` loads scikit-learn standard scalers (`scaler_X.pkl` and `scaler_y.pkl`) using `joblib` and calls `.transform()` to normalize model features.
* **The Issue:** Importing `joblib` and scikit-learn (`sklearn`) consumes over 150MB of RAM and takes 2-3 seconds at startup, which is highly inefficient on the resource-constrained Jetson Orin.
* **Proposed Enhancement:**
  - Export the mean, standard deviation, scale, and min/max parameters of the training scalers as a lightweight `.npy` or `.json` file.
  - Perform the scaling arithmetic directly in raw Python/NumPy: $x_{scaled} = (x - \mu) / \sigma$.
  - This completely removes scikit-learn and joblib dependencies from `velocity_estimator.py`, saving 150MB of memory and accelerating startup.

---

## [2026-06-06 03:00:00 -07:00] Iteration 20 Analysis

### 73. Multi-Scale Temporal Resolution Input Feature Vector
* **Context:** The feature vector built in `_build_window_features` utilizes a fixed 10-frame window representing the last 1.0 second of history sampled consecutively at 10Hz.
* **The Issue:** A single uniform resolution does not scale well: short-term windows fail to capture long-term walking trends, while long-term windows average out sudden pace changes or stops.
* **Proposed Enhancement:**
  - Build a multi-scale feature vector of the same fixed size $(1, 40)$, but partition it to capture multiple temporal resolutions.
  - Dedicate 5 entries to the short-term scale (last 5 consecutive frames at 10Hz), 5 to the medium-term scale (alternate frames spanning 1.8 seconds), and 5 to the long-term scale (every third frame spanning 3.0 seconds).
  - This informs the model of both high-frequency local acceleration and overall trajectory patterns.

### 74. Boundary-Truncation Reconstruction and Shape Fitting
* **Context:** In `_extract_depth_centroids`, centroids of close-range obstacles are calculated using binary image moments.
* **The Issue:** When a pedestrian stands extremely close to the camera, their body contour is truncated by the edges of the image frame. Standard image moments skew the centroid coordinate toward the center of the visible frame, corrupting estimated displacements.
* **Proposed Enhancement:**
  - Detect if a contour intersects the image borders ($x = 0, y = 0,$ etc.).
  - If boundary intersection is detected, fit a geometric ellipse or human head-and-shoulders model to the non-truncated contour boundaries.
  - Extrapolate the truncated coordinates beyond the frame boundary to reconstruct the full shape before computing the centroid, eliminating truncation bias.

### 75. Velocity Obstacle (VO) Vector Selection for Holonomic Navigation
* **Context:** In `ab_comparison_test.py`, the robot halts forward movement (`speed = 0.0`) and sets `is_paused = True` when the lateral bypass path is blocked.
* **The Issue:** Halting linear velocity whenever an obstacle enters the diagonal bypass corridor causes blocky movements and increases traversal time.
* **Proposed Enhancement:**
  - Implement a local Velocity Obstacle (VO) selection algorithm. Represent obstacles as collision cones in the robot's velocity space.
  - Search for and select a combined velocity vector $(v_x, v_y)$ that is closest to the target waypoint vector but lies outside the active collision cones.
  - This lets the robot dynamically steer around dynamic obstacles using continuous velocity angle corrections, avoiding abrupt halts.

### 76. Decoupled Background YOLO Throttling with Velocity Projection
* **Context:** In `broadcast_loop` of `server_x3.py`, YOLOv11 object detection runs on every camera frame at 20Hz.
* **The Issue:** Running deep learning inference at 20Hz on the Jetson Orin keeps the GPU under constant heavy load, causing thermal throttling and starving critical navigation threads.
* **Proposed Enhancement:**
  - Run YOLO in a background executor capped at 5Hz to find obstacles.
  - For intermediate frames, update/project the bounding box positions using the pedestrian's estimated velocity vector ($vx, vy$) provided by the 10Hz estimator.
  - This cuts GPU utilization by 75% while maintaining a high-frequency visualization overlay.

### 77. Connected Components with Stats for Centroid Extraction
* **Context:** Centroids are extracted in `velocity_estimator.py` by finding contours and iterating over them in Python to calculate areas and moments.
* **The Issue:** If the depth frame is noisy or contains textured surfaces (like carpets), `findContours` returns dozens of small contours. Iterating over them and computing image moments in Python loops is slow.
* **Proposed Enhancement:**
  - Replace `cv2.findContours` and subsequent Python loops with OpenCV's C++ optimized `cv2.connectedComponentsWithStats()`.
  - This returns areas, bounding boxes, and centroids of all labeled regions in a single optimized pass.
  - Filter out small regions using a fast vector mask in NumPy, eliminating Python loops and moments calculations entirely.

---

## [2026-06-06 04:00:00 -07:00] Iteration 21 Analysis

### 78. Kinematic Acceleration-Limiting Output Filter
* **Context:** The MLP model output provides real-time velocity estimates ($v_x, v_y$) for tracked obstacles.
* **The Issue:** Under tracking jitter or lighting variance, the MLP output velocities can spike or change direction instantaneously, resulting in physically impossible acceleration profiles (e.g. $>5$ m/s$^2$) and jerky speed-scaling reactions.
* **Proposed Enhancement:**
  - Apply a kinematic acceleration-limiting filter to the predicted velocity outputs of each track.
  - Limit the implied acceleration between consecutive steps to a maximum human capability (e.g., $a_{max} = 3.0\text{ m/s}^2$). If the acceleration magnitude exceeds this limit, cap it and compute the smoothed velocity command: $v_{smooth} = v_{prev} + a_{capped} \cdot dt$.
  - This enforces physical plausibility on the output velocities, smoothing out navigation responses.

### 79. Semantic Gating with YOLO Bounding Boxes
* **Context:** The `VelocityEstimator` tracks all depth-contour centroids within the specified range limit.
* **The Issue:** Dynamic lighting, dust, and specular shadows in tight classroom areas create false depth centroids. The tracker treats these as dynamic obstacles, triggering false deceleration.
* **Proposed Enhancement:**
  - Project the 3D depth-contour centroids onto the camera image plane and check for spatial overlap with YOLO bounding boxes of dynamic classes (e.g., `person`).
  - Require a centroid track to maintain semantic overlap with a verified YOLO bounding box for at least 3 frames before designating it as a dynamic obstacle for speed scaling.
  - This prevents static clutter or shadow noise from triggering false bypass/braking maneuvers.

### 80. Rear-Sector LiDAR Protection for Backing Recovery
* **Context:** In `ab_comparison_test.py`, the active recovery behavior executes a 1.5-second backing maneuver at $-0.08\text{ m/s}$ if the robot is blocked for $>8.0$ seconds.
* **The Issue:** The depth camera and forward-sector LiDAR block checks only scan the forward path. The robot is blind to obstacles behind it during the backing recovery, risking collisions with walls or bystanders.
* **Proposed Enhancement:**
  - Query the rear LiDAR scan sector (angles from $135^\circ$ to $225^\circ$ in the robot frame) before and during the backing recovery.
  - If any obstacle is detected within $0.35$m of the robot's rear chassis, immediately halt the backing maneuver. This adds spatial safety to the recovery loop.

### 81. Immediate Depth Downsampling for Centroid Extraction
* **Context:** `velocity_estimator.py` processes raw depth images at full resolution ($640 \times 480$) to find obstacle contours and compute centroids.
* **The Issue:** Operations on 307,200 pixels consume significant memory bandwidth and CPU cycles, which is unnecessary since pedestrians are large shapes.
* **Proposed Enhancement:**
  - Resize the raw depth image immediately upon receipt to $320 \times 240$ or $160 \times 120$ using nearest-neighbor interpolation.
  - Run all downstream thresholding, masking, and component extraction on the downsampled grid. This cuts CPU processing time and memory allocations by 75% to 90% without degrading spatial accuracy.

### 82. Dynamic Subscription Lifecycle Management for Idle Power Savings
* **Context:** The `ROS2Bridge` in `server_x3.py` maintains image and depth image topic subscribers active at all times to receive sensor data.
* **The Issue:** Deserializing and processing 20Hz image streams via ROS2/FastDDS consumes substantial CPU (10-15% of a Jetson core) even when no browser GUI client is connected and the GUI is idle.
* **Proposed Enhancement:**
  - Monitor the number of connected WebSocket clients.
  - Dynamically instantiate the image and depth subscribers when `connected_clients` transitions from 0 to 1, and destroy the subscribers when the count returns to 0.
  - This stops image deserialization overhead when idle, preserving CPU/power on the Jetson Orin.

---

## [2026-06-06 05:00:00 -07:00] Iteration 22 Analysis

### 83. Hardware-Triggered Pose-Timestamp Interpolation
* **Context:** EKF odometry coordinates from ROS2 are used to compensate for robot ego-motion on the tracked pedestrian centroids.
* **The Issue:** Small delays between camera frame capture and ROS2 odom message callbacks (typical latency of 10-30ms) cause coordinate transformations to use slightly lagged poses. This mismatch creates tracking jitter and "ghost speeds" on static objects.
* **Proposed Enhancement:**
  - Maintain a sliding queue of EKF odom updates keyed by their exact ROS header timestamp.
  - When transforming depth centroids, extract the depth frame's capture timestamp and linearly interpolate the robot's pose between the two nearest odom frames in the queue.
  - Sychronizing odom pose to the exact moment of camera frame capture eliminates latency-induced tracking jitter and improves velocity estimation accuracy.

### 84. Edge-Proximity Confidence Weighting for FOV Transitions
* **Context:** Pedestrians are tracked as they enter and exit the camera's horizontal field of view.
* **The Issue:** When a pedestrian is partially cut off at the left/right frame borders, their visible shape shifts, producing large false horizontal displacement steps ($dy$) that lead to massive spikes in the estimated velocity.
* **Proposed Enhancement:**
  - Track the contour's pixel distance to the left and right borders of the image frame.
  - If a contour enters a narrow border zone (e.g. within 25 pixels of the frame edge), flag it as an "unstable-edge" state.
  - While in this state, decrease tracking matching confidence, and clamp coordinate displacements to their last stable velocity vector rather than using raw boundary coordinates.

### 85. Predictive Dynamic Corridor Shifting
* **Context:** In `ab_comparison_test.py`, the robot sets a static lateral offset target ($\pm 0.25\text{ m}$ or $\pm 0.45\text{ m}$) to bypass obstacles.
* **The Issue:** If the pedestrian is walking diagonally towards the robot's bypass side, the static offset target will lead to a collision. The controller fails to adapt to the obstacle's lateral velocity vector.
* **Proposed Enhancement:**
  - Dynamically update `target_lateral_offset` in every frame by adding a projection of the pedestrian's lateral velocity: $\text{offset\_target}(t) = \text{pedestrian\_y}(t) \pm (\text{pedestrian\_width} + \text{safety\_margin}) + v_{y,pedestrian} \cdot \text{lookahead\_time}$.
  - This guides the robot to steer towards the side that is *getting clearer* rather than committing to a blocked path.

### 86. Adaptive Duty-Cycle Throttling in Estimator Idle States
* **Context:** The `VelocityEstimator` executes range thresholding, contour finding, and MLP inference at a constant 10Hz.
* **The Issue:** When the robot is stopped (e.g. settling between segments) or when no tracks have been active for a long period, running full-rate processing wastes CPU and battery.
* **Proposed Enhancement:**
  - Monitor the robot's linear velocity and track history.
  - If the robot is stationary and no dynamic tracks are active for $>5.0$ seconds, automatically transition the estimator to an idle state, throttling the execution frequency to 2Hz.
  - Instantly spin the frequency back to 10Hz as soon as a movement command is received or a change in the LiDAR scan is detected.

### 87. High-Performance orjson Serialization
* **Context:** In `server_x3.py`'s 20Hz `broadcast_loop`, complex dictionaries containing float arrays (`velocity_estimates`, `detections`, `robot_pose`) are converted to JSON text using Python's standard `json` module.
* **The Issue:** Python's built-in `json.dumps` is relatively slow, especially when serializing nested float arrays at high frequencies, adding CPU overhead to the broadcast loop.
* **Proposed Enhancement:**
  - Replace the standard `json` module with `orjson` (a highly optimized Rust-based JSON library).
  - Use `orjson.dumps(msg)` which serializes float arrays and NumPy variables directly without intermediate `.tolist()` conversions, reducing message serialization latency.

---

## [2026-06-06 06:00:00 -07:00] Iteration 23 Analysis

### 88. Heading-Aligned Path Rotation for Trajectory Invariance
* **Context:** Feature sequences built in `_build_window_features` feed relative displacements $dx, dy$ in the camera frame directly to the MLP model.
* **The Issue:** Displacements vary depending on the absolute heading angle at which the pedestrian is crossing the camera field of view, requiring the neural network to generalize across arbitrary walk angles, which hurts regression accuracy.
* **Proposed Enhancement:**
  - Calculate the overall path heading vector of the pedestrian over the active window: $\mathbf{w} = [x_{last} - x_{first}, y_{last} - y_{first}]^T$ and $\theta = \text{atan2}(w_y, w_x)$.
  - Rotate all coordinate displacements in the history window by $-\theta$. This aligns the displacements so that the pedestrian's overall direction of motion always points along the positive x-axis of the feature space.
  - Feeding these aligned vectors (plus the path angle $\theta$ as a separate feature) isolates the shape and speed profile from the absolute walking direction, significantly improving prediction accuracy.

### 89. Ground Plane RANSAC Filtering for Leg Contour Isolation
* **Context:** In `_extract_depth_centroids`, obstacle contours are extracted from range-thresholded binary depth masks.
* **The Issue:** In tight areas or during robot pitch oscillations, the floor plane enters the bottom sector of the depth frame, merging the ground into the pedestrian's leg contours. This shifts centroids downward and dampens estimated displacements.
* **Proposed Enhancement:**
  - Fit a ground plane model ($aX + bY + cZ + d = 0$) to the bottom sector points of the raw depth frame in every cycle using a fast RANSAC solver.
  - Calculate the height of each depth point relative to this ground plane.
  - Filter out pixels lying close to the ground (e.g. $<0.05$m) before contour extraction, isolating clean, upright human legs and furniture legs.

### 90. Hysteresis-Based Speed Scaling for Deceleration Smoothing
* **Context:** In `ab_comparison_test.py`, the linear speed command is modulated by the estimated Time-to-Collision (TTC) scaling factor $s_t = (ttc - \text{TTC\_MIN}) / (\text{TTC\_THRESHOLD} - \text{TTC\_MIN})$.
* **The Issue:** When the robot brakes to avoid a pedestrian, the relative velocity decreases, which causes the calculated TTC to increase. This triggers the robot to accelerate again, leading to speed oscillation (braking chatter).
* **Proposed Enhancement:**
  - Implement a hysteresis filter or a directional low-pass filter on the speed scaling factor $s_t$.
  - Allow the scaling factor to drop instantly (fast braking) but restrict its increase (slow recovery acceleration) using a lag filter until the minimum distance to the obstacle has started to increase. This ensures smooth, oscillation-free decelerations.

### 91. Vectorized Pairwise Distance Matrix Broadcasting
* **Context:** `ObstacleTracker.update` calculates Euclidean distances between new centroids and existing tracks using list comprehensions and loops in Python.
* **The Issue:** Sequential distance queries in Python loops create interpreter overhead that scales with the number of dynamic tracks, adding latency in crowded rooms.
* **Proposed Enhancement:**
  - Convert active tracks and new centroids into NumPy arrays of shape $(M, 2)$ and $(N, 2)$.
  - Calculate the pairwise Euclidean distance matrix $D$ of shape $(M, N)$ in a single vectorized NumPy broadcasting operation: $D = \text{np.linalg.norm}(\text{tracks}[:, \text{np.newaxis}, :2] - \text{centroids}[\text{np.newaxis}, :, :2], \text{axis}=2)$.
  - Global matches are then selected from this matrix, completely bypassing Python loop overhead.

### 92. Pre-Allocated LUT Map Pixel Mapping
* **Context:** In `_encode_mapu` of `server_x3.py`, OccupancyGrid values (-1 to 100) are mapped to grayscale pixels (128, 255, 0) using three separate boolean indexing operations on the map array.
* **The Issue:** Boolean indexing allocates three large boolean masks on every map push, consuming CPU cycles and memory bandwidth when processing large maps.
* **Proposed Enhancement:**
  - Pre-allocate a 256-element NumPy array lookup table (LUT) mapping grid data values to target grayscale values (index 0 maps to 255, 1-100 map to 0, and others map to 128).
  - Perform the pixel mapping using direct array indexing: `pixels = LUT[data]`. This executes the translation natively in C++ in a single pass with zero boolean mask allocations, speeding up map encoding by 5x.

---

## [2026-06-06 07:00:00 -07:00] Iteration 24 Analysis

### 93. MLP-Velocity-Driven Dead Reckoning for Occluded Tracks
* **Context:** If a tracked pedestrian is temporarily occluded by a desk or table, the track receives no updates but is kept alive for up to `max_age = 10` frames.
* **The Issue:** When the pedestrian is occluded, coordinate updates stop, leaving gaps in the track's sequence history window. Upon reappearance, the displacement $dx$ is calculated over the entire multi-frame gap, creating false speed spikes.
* **Proposed Enhancement:**
  - If a track is not matched in a frame, perform dead reckoning using its last estimated velocity: $x_t = x_{t-1} + v_{x,last} \cdot dt$ and $y_t = y_{t-1} + v_{y,last} \cdot dt$.
  - Append these predicted coordinates to the sequence history queue on each missing frame. This maintains window continuity and preserves clean sequence shapes for the MLP.

### 94. Spatio-Temporal Static Saliency Masking for Doorways
* **Context:** `_extract_depth_centroids` extracts all centroids in the range of 0.5m to 4.0m.
* **The Issue:** When navigating through doorways or narrow corridors, the side doorframes and partition panels enter the threshold range, producing false dynamic obstacle tracks that block the robot from driving through.
* **Proposed Enhancement:**
  - Build a local spatio-temporal saliency map. If a track's global velocity is consistently near zero ($<0.05$m/s) for $>3.0$ seconds, classify it as a static structural fixture.
  - Project this structural coordinate back into the image plane and add it to an image-space exclusion mask.
  - Exclude masked regions from thresholding in future frames, allowing the robot to ignore static doorways.

### 95. Dynamic Side-Clearance Potential Fields
* **Context:** In `ab_comparison_test.py`, the linear speed command is scaled by forward blockages, and side clearances are checked using static thresholds.
* **The Issue:** Proportional-only side checks are binary; the robot does not naturally slow down or veer away from walls as it gets closer, resulting in blocky strafes and side-swipe risks.
* **Proposed Enhancement:**
  - Define a repulsive artificial potential field force $F_{rep} = \frac{\eta}{(d - d_{min})^2} \cdot \mathbf{n}$ originating from obstacles in the left and right LiDAR sectors.
  - Add this repulsive force directly to the lateral velocity command `vy_cmd` as a correction term.
  - This lets the robot naturally drift away from walls or desk corners as it gets near them, creating a continuous protective cushion of safety.

### 96. Precomputed Pinhole Projection Grid (Projection LUT)
* **Context:** In `_extract_depth_centroids`, pixel coordinates $(cx, cy)$ are projected to physical meters $(x_m, y_m)$ using division and multiplication operations in Python: `x_m = (cx - w / 2.0) * Z / fx`.
* **The Issue:** Performing float divisions and coordinate centering for multiple centroids in Python loops on every frame is inefficient.
* **Proposed Enhancement:**
  - Precompute two static float32 lookup tables `PX_LUT` and `PY_LUT` matching the dimensions of the downsampled depth frame: `PX_LUT[v, u] = (u - cx) / fx`.
  - Obtain physical coordinates instantly using index lookups and a single multiplication: `x_m = PX_LUT[cy, cx] * Z` and `y_m = PY_LUT[cy, cx] * Z`.
  - This removes all float division operations per frame, replacing them with fast array accesses.

### 97. Decoupled High/Low Frequency Telemetry Split
* **Context:** The WebSocket server `server_x3.py` sends a unified JSON readout containing pose, sensor logs, models, and UI configuration settings at 20Hz.
* **The Issue:** Telemetry files contain many static configuration values that do not change from frame to frame, wasting serialization time and network bandwidth.
* **Proposed Enhancement:**
  - Split the WebSocket telemetry into a High-Frequency (HF) stream and a Low-Frequency (LF) stream.
  - Send HF streams (pose, battery, tracking estimates, encoders) at 20Hz.
  - Send LF configuration settings (active models, options, mode states) at 1Hz or only upon change events, cutting the average JSON payload size by 60%.

---

## [2026-06-06 08:00:00 -07:00] Iteration 25 Analysis

### 98. Barycentric Relative Coordinate Stabilization
* **Context:** Pedestrian tracking coordinates are transformed from camera coordinates to the global map frame using EKF odometry updates to compensate for ego-motion.
* **The Issue:** During rapid turnaround spins ($180^\circ$ turns), small localization errors and angular heading lag in the EKF odometry create significant coordinate projection jumps. This induces "ghost speeds" on static reference points.
* **Proposed Enhancement:**
  - Locate the nearest static cluster in the LiDAR scan (e.g. a wall corner or desk leg) and designate it as a local coordinate reference anchor.
  - Compute the coordinates of the pedestrian relative to this anchor.
  - Since both the anchor and the pedestrian are subject to the same ego-motion, subtracting the anchor position cancels out all robot rotation and translation errors, providing noise-free tracking sequences during high-speed turnaround spins.

### 99. Depth Histogram Peak-Slicing for Dynamic Segmenting
* **Context:** Obstacles in `_extract_depth_centroids` are segmented using a hardcoded depth range threshold of $0.5\text{ m} \le Z \le 4.0\text{ m}$.
* **The Issue:** In tight hallways, static partitions, tables, or cabinets fall directly inside this range, producing large, cluttered masks that merge with the pedestrian's legs and corrupt centroid coordinate tracking.
* **Proposed Enhancement:**
  - Compute a 1D histogram of raw depth values in the forward camera frustum.
  - Identify distinct depth peaks in the histogram, which represent the exact spatial depth locations of isolated objects (pedestrians, furniture).
  - Dynamically set the thresholding bounds to wrap tightly around these peaks (e.g. $\text{peak\_depth} \pm 0.35$m), separating dynamic pedestrians from background clutter.

### 100. Spatio-Temporal Gap Selection (Window of Opportunity)
* **Context:** In `ab_comparison_test.py`, the speed scaling and lateral offset controllers react only when a pedestrian blocks the immediate forward corridor.
* **The Issue:** Reactive avoidance leads to late steering corrections and sudden braking. The robot cannot plan ahead when navigating crowds or narrow doors.
* **Proposed Enhancement:**
  - Project the future trajectories of all tracked dynamic obstacles over a 3.0-second horizon.
  - Identify spatial "valleys" (regions of space-time where no predicted pedestrian paths intersect).
  - Select the optimal lane corridor (left, right, or center) that offers the longest uninterrupted window of opportunity, steering the robot proactively before collision conflicts occur.

### 101. Pre-Allocated NumPy Ring Buffers for Coordinate History
* **Context:** In `_build_window_features`, the track history list/deque is converted to a NumPy array on every iteration, which is then reshaped to generate the $(1, 40)$ feature vector.
* **The Issue:** Allocating new NumPy arrays and copying history sequences at 10Hz for multiple tracks wastes memory and interpreter cycles on the Jetson.
* **Proposed Enhancement:**
  - Pre-allocate a single flat NumPy array of shape $(\text{MAX\_OBSTACLES}, \text{WINDOW\_SIZE}, 3)$ at initialization to store all track coordinates.
  - Maintain a rolling head pointer index. Update history by writing new centroids directly into the pre-allocated slice coordinates.
  - Generate features by slicing and reshaping, which creates a memory **view** rather than copying the array, eliminating allocation overhead.

### 102. Event-Driven Camera-Synced Broadcast Loop
* **Context:** In `server_x3.py`, the `broadcast_loop` queries camera frames and telemetries at a fixed 20Hz cap using `asyncio.sleep(0.05)`.
* **The Issue:** The camera captures frames at 30Hz. A fixed 20Hz timer cap results in timing drift, where the server periodically processes duplicate frames or skips frames, wasting processing cycles.
* **Proposed Enhancement:**
  - Synchronize the broadcast loop directly to the camera frame capture thread.
  - Use an `asyncio.Event` to trigger the broadcast loop immediately after a new camera frame is written to memory.
  - This ensures the server only processes unique frames, eliminating redundant JPEG compressions and reducing idle CPU load.

---

## [2026-06-06 09:00:00 -07:00] Iteration 26 Analysis

### 103. Model-Prediction Autoregressive Feedback Loop
* **Context:** The MLP model processes historical displacement sequences to output the pedestrian's current velocity.
* **The Issue:** A simple feedforward network has no internal memory of its own past predictions. An isolated centroid measurement noise spike immediately distorts the output velocity, even if the pedestrian has walked at a constant speed for several seconds.
* **Proposed Enhancement:**
  - Feed the model's past velocity predictions ($v_{t-1}, v_{t-2}, v_{t-3}$) directly back into the input feature vector as autoregressive features.
  - This allows the model to leverage its own short-term prediction history to stabilize state transitions, smoothing out coordinate measurement noise and improving prediction robustness during sudden pace changes.

### 104. Temporal Inter-Frame Depth Differencing (Motion Masking)
* **Context:** Depth thresholding is applied directly to raw depth frames to find obstacle contours.
* **The Issue:** In tight hallways, static partitions, doorway jambs, and cabinet edges fall inside the depth range, producing false dynamic tracks that trigger unnecessary evasive actions.
* **Proposed Enhancement:**
  - Compute the absolute difference between the current depth frame and the depth frame from 0.2 seconds ago.
  - Apply range-rate thresholding to isolate dynamic pixels (depth changes $>0.05$m) and mask out static background structures.
  - Restrict the contour extraction and centroid tracking pipeline to run only within this motion mask. This filters out static furniture and door frames.

### 105. Active Center-Point Drift Correction during Rotation
* **Context:** In `ab_comparison_test.py`, the robot performs turnaround maneuvers (`ROTATE_180` and `ROTATE_HOME`) in place by commanding angular velocity.
* **The Issue:** Pure in-place rotations on mecanum wheels induce substantial wheel slip, causing the robot's physical center point to drift. When rotation is complete, the robot starts its next drive segment offset from the path centerline.
* **Proposed Enhancement:**
  - Track the robot's current coordinates relative to the starting position $(x_0, y_0)$ of the rotation state using EKF odometry.
  - If drift occurs during the spin, inject minor corrective lateral and forward linear speed terms (`linear.x` and `linear.y` Twist commands) to actively hold the robot's center point at $(x_0, y_0)$ while it rotates.
  - This eliminates rotational drift, keeping the robot aligned with the centerline and maximizing wall clearance.

### 106. Downscaled Depth Visualization Encoding
* **Context:** The server encodes depth frames at full resolution ($640 \times 480$) to JPEG and sends them over WebSockets when depth visualization is active.
* **The Issue:** Running JPEG compression on a 640x480 array at 10Hz consumes substantial CPU bandwidth on the Jetson Orin to produce a feed that is only displayed in a small preview panel in the Web GUI.
* **Proposed Enhancement:**
  - Downscale the depth frame to $320 \times 240$ using fast nearest-neighbor scaling before running JPEG compression.
  - This decreases JPEG compression execution time by over 75% while keeping the visual quality in the GUI preview panel identical.

### 107. Double Exponential Smoothing (Holt's Linear Trend)
* **Context:** Relative displacements ($dx, dy$) in `velocity_estimator.py` are calculated as raw differences between adjacent centroid frames.
* **The Issue:** Astra depth resolution limits cause coordinate displacements to be highly discretized, introducing high-frequency noise into the MLP feature vectors.
* **Proposed Enhancement:**
  - Apply Double Exponential Smoothing (Holt's linear trend algorithm) to the coordinate sequences before feature extraction.
  - Holt's algorithm smoothing separates coordinate level tracking from velocity trend calculations. This filters out high-frequency spatial discretization noise, feeding high-fidelity floating-point trend vectors to the MLP.

---

## [2026-06-06 10:00:00 -07:00] Iteration 27 Analysis

### 108. Kinematic Stop-Trigger Gating
* **Context:** Pedestrian velocities are estimated using coordinate sequences spanning a 1.0-second history window.
* **The Issue:** When a walking pedestrian stops abruptly, the history window still contains non-zero displacements from earlier frames, causing the MLP output to predict a non-zero speed for up to 1 second. This lag makes the robot remain paused/slowed for too long after the path has cleared.
* **Proposed Enhancement:**
  - Check the raw displacements ($dx, dy$) of the last 3 consecutive frames in the active window.
  - If the displacements fall below a tiny noise threshold ($<0.01$m), bypass the MLP inference and force the output estimated velocity to $(0.0, 0.0)$ immediately.
  - This hard-gating eliminates neural network lag when a pedestrian halts, accelerating recovery times.

### 109. Centroid Presence Verification Filter (Track Initiation Gate)
* **Context:** The `ObstacleTracker` instantiates a new track immediately upon receiving a centroid that doesn't match an active track.
* **The Issue:** Specular reflections, dust, or background shadows in tight classrooms create brief, transient depth centroids. Instantiating tracks immediately for these transient points pollutes the list and triggers false evasive braking.
* **Proposed Enhancement:**
  - Introduce a track candidate list for new, unmatched centroids.
  - Require a candidate centroid to be detected in at least 3 out of 5 consecutive frames before upgrading it to an active track.
  - This track initiation gate filters out transient noise blocks, ensuring that velocity estimation is restricted to verified dynamic obstacles.

### 110. Dynamic Corridor Width Clearance Scaling
* **Context:** In `ab_comparison_test.py`, the lateral bypass offset commands a fixed displacement value ($\pm 0.25\text{ m}$ or $\pm 0.45\text{ m}$) relative to the centerline.
* **The Issue:** Classroom layouts can have variable-width hallways. Commanding a fixed lateral offset in a narrowing corridor can drive the robot directly into the side walls.
* **Proposed Enhancement:**
  - Calculate the current corridor width dynamically in every frame using the sum of the left and right LiDAR clearance ranges: $w_{corr} = d_{left} + d_{right}$.
  - Scale the maximum allowed bypass offset targets proportionally to the corridor width: $\text{OFFSET}_{max} = \text{clamp}(0.5 \cdot (w_{corr} - w_{robot}), 0, \text{BYPASS\_OFFSET})$.
  - This shrinks the bypass envelope in narrow choke points, preventing side-wall collisions.

### 111. Serial Command Write Rate Optimization (30Hz Throttle)
* **Context:** In `server_x3.py`, motor velocity commands are written to the serial ROSMASTER board interface at a high-frequency 100Hz rate.
* **The Issue:** Mecanum motors have mechanical inertia and cannot react to 100Hz updates. Flooding the serial port at 100Hz causes high CPU utilization in serial write blocks and can saturate the controller buffer, adding response lag.
* **Proposed Enhancement:**
  - Reduce the Rosmaster serial write update rate inside `motion_loop` from 100Hz to 30Hz (`asyncio.sleep(0.033)`).
  - This decreases serial write CPU overhead by 70% and reduces serial port lock contention, leaving more processing time for tracking nodes.

### 112. Single-Precision float32 Scale Arithmetic
* **Context:** Scalar parameters ($\mu, \sigma$) for training standardizations are loaded from a JSON configuration file, and normalization math is computed in `velocity_estimator.py`.
* **The Issue:** Loading parameters as standard Python floats leads to double-precision `float64` NumPy arrays. Converting these arrays to `float32` tensors on every frame adds minor CPU casting overhead.
* **Proposed Enhancement:**
  - Pre-cast the loaded parameter arrays to single-precision float32: `mean = np.array(mean, dtype=np.float32)`.
  - Perform all subsequent scaling math in float32. This keeps the data types uniform and avoids casting overhead before tensor instantiation.

---

## [2026-06-06 11:00:00 -07:00] Iteration 28 Analysis

### 113. Edge-Preserving Bilateral Filtering for Centroid Stability
* **Context:** Pedestrian centroids are calculated from binary threshold masks derived from raw depth frame matrices.
* **The Issue:** Depth sensor pixel noise and surface shadows create high-frequency boundary fluctuations in extracted contours, causing the calculated centroids to drift. This introduces jitter noise into coordinate displacement features.
* **Proposed Enhancement:**
  - Apply an edge-preserving **Bilateral Filter** (or a fast Median Filter) to the raw depth image before thresholding.
  - Unlike Gaussian blur, the bilateral filter smooths out pixel noise inside the pedestrian's body while preserving sharp depth edges at boundaries. This stabilizes contour boundaries and reduces coordinate displacement jitter.

### 114. Vertical Wall RANSAC Filtering
* **Context:** In `_extract_depth_centroids`, obstacle range thresholding is performed to find candidates.
* **The Issue:** In narrow corridors, the side walls fall inside the threshold range, producing large static contours that merge with pedestrians.
* **Proposed Enhancement:**
  - Implement a vertical plane RANSAC extraction step on the left and right sectors of the raw depth cloud.
  - Fit side-wall equations: $A_w X + B_w Y + C_w Z + D_w = 0$.
  - Filter out points close to these vertical planes (e.g. within $0.08$m) before contour finding, suppressing static corridor boundaries.

### 115. Live LiDAR-Wall Relative Alignment at Waypoint A
* **Context:** In `ab_comparison_test.py`, the robot drives back to Waypoint A and stops when its EKF-fused odometry reaches coordinates $(0,0)$.
* **The Issue:** Turnaround slippage and gyro drift accumulate, causing the EKF pose to drift from the physical tape mark. The robot fails to stop at the exact start spot over consecutive runs.
* **Proposed Enhancement:**
  - Record the robot's initial relative distance to the rear (or front) wall using the LiDAR scan at the start of the test.
  - During the return segment (`DRIVE_TO_A`), instead of stopping strictly based on EKF odometry coordinates, use the LiDAR to measure the wall distance.
  - Adjust the final stopping command to match the initial reference wall distance. This compensates for EKF drift at the end of every run.

### 116. Pre-Allocated PyTorch Input Tensor Reuse
* **Context:** In `velocity_estimator.py`, a new PyTorch tensor is allocated on every frame for each active track before running MLP inference.
* **The Issue:** Instantiating new tensor objects at 10Hz creates heap allocation overhead and increases CPU memory bandwidth consumption.
* **Proposed Enhancement:**
  - Pre-allocate a single PyTorch tensor of shape $(\text{MAX\_OBSTACLES}, 40)$ at startup.
  - In each step, copy features directly into the pre-allocated memory: `x_tensor_preallocated[i].copy_(torch.from_numpy(features_scaled))`.
  - Execute inference on a sliced view of the pre-allocated tensor: `self._model(x_tensor_preallocated[:num_tracks])`, avoiding runtime heap allocations.

### 117. Non-Blocking WebSockets Payload Compression
* **Context:** In `server_x3.py`, the `websockets` library is used to push map updates and camera JPEGs to the GUI.
* **The Issue:** If `permessage-deflate` compression is enabled, the library runs a blocking zlib compression on the main thread. Compressing large map payloads blocks the Python event loop for 10-20ms, dropping camera frames and delaying velocity commands.
* **Proposed Enhancement:**
  - Disable standard `permessage-deflate` on the WebSocket server for large binary payloads.
  - Run compression in a background executor pool using `loop.run_in_executor` with a fast algorithm like `lz4` or `zstd`, or transmit uncompressed binary frames. This prevents event loop thread blockage and guarantees stable motor command latencies.

---

## [2026-06-06 12:00:00 -07:00] Iteration 29 Analysis

### 118. Spatio-Temporal Cluster Merging for Multi-Scale Blob Extraction
* **Context:** `_extract_depth_centroids` uses a single `MIN_BLOB_AREA = 500` threshold to filter out tiny blobs.
* **The Issue:** When the robot turns or moves into narrow/tight areas, dynamic obstacles (like a person's legs or a torso partially blocked by desks) are split into multiple smaller blobs that are individually < 500 pixels. This causes feature detection failures and tracking dropouts.
* **Proposed Enhancement:**
  - Instead of discarding blobs under `MIN_BLOB_AREA` immediately, keep smaller blobs (e.g. > 150 pixels) if they are close to each other in 3D space (e.g. Euclidean distance < 0.35m) and merge their contours/masks.
  - This spatio-temporal merging before centroid calculation restores detection accuracy in narrow spaces where legs or bodies are partially occluded or split by close furniture.

### 119. Adaptive Lidar-Based Dynamic Scan Angle Filtering
* **Context:** `_update_bypass_offset` scans the lidar sector using static angle boundaries (e.g. `abs(angle) < 0.52` for front block checks).
* **The Issue:** When turning or driving on curved paths, a static forward sector does not align with the robot's actual instantaneous path (the trajectory curve). This causes the robot to either miss obstacles it is about to turn into, or falsely detect walls/obstacles that are no longer in its path.
* **Proposed Enhancement:**
  - Dynamically skew and size the lidar scan angles based on the current angular velocity `omega` (from odometry).
  - Calculate the center of the arc of travel $R = v / \omega$ and define the forward block checking sector as a curved bounding box matching the swept path of the robot.
  - This prevents false collision triggers on static walls during sharp turns in narrow spaces while ensuring obstacles along the path of travel are detected.

### 120. Target-Centric Kalman Filter Gate for WebSocket Telemetry Bandwidth Reduction
* **Context:** `server_x3.py` sends `velocity_estimates` for all tracked objects over the WebSocket to `ab_comparison_test.py` at 20Hz.
* **The Issue:** Standard network payloads include stationary objects or distant targets that the robot's local planner does not need to react to. This causes unnecessary network transmission overhead and increases CPU overhead on the main thread processing the JSON readouts.
* **Proposed Enhancement:**
  - Implement a spatial gating filter on the server before broadcasting estimates.
  - Only include estimates in the WebSocket readout if they lie within a $2.5\text{m}$ radius around the robot or have a positive heading vector pointing towards the robot's projected trajectory corridor.
  - This cuts down WebSocket serialization/deserialization latency and JSON parsing CPU load in the test scripts.

### 121. Vectorized Centroid Global Transformation via Matrix Dot Products
* **Context:** In `velocity_estimator.py`, global coordinates for centroids are computed in a Python `for` loop, calculating trigonometric functions and offsets for each centroid one by one.
* **The Issue:** Executing loop-based coordinate transforms in Python at 10Hz introduces latency, especially when there are multiple contours or LiDAR clusters.
* **Proposed Enhancement:**
  - Vectorize the transformation using NumPy. Stack local centroids `(cz, -cx)` into a homogeneous coordinate matrix of shape $(N, 3)$.
  - Construct a single 2D homogeneous transformation matrix $T_{robot}^{global}$ using the robot's current pose.
  - Compute the global coordinates in a single vectorized matrix multiplication: `centroids_g = (T_robot_global @ local_coords.T).T`. This avoids all Python loops and trigonometric calls per centroid, maximizing execution efficiency.

### 122. Dynamic Path Deceleration via Predictive Time-to-Collision (TTC) Hysteresis
* **Context:** In `ab_comparison_test.py`, `_get_speed_scaling` computes speed scaling factor and instantly applies it to the commanded speed.
* **The Issue:** Instantaneous scaling based on noisy velocity estimates leads to jerky speed commands (velocity chattering), causing wheel slip and robot vibration.
* **Proposed Enhancement:**
  - Implement a hysteresis filter or soft-limit ramp on the speed scaling factor `speed_scale`.
  - Define an EMA-smoothed speed scale: `self.smooth_speed_scale = beta * speed_scale + (1 - beta) * self.smooth_speed_scale`.
  - If a sudden decrease in speed scale is commanded (e.g., from 1.0 to 0.2), allow it to drop rapidly for safety, but if the path clears (scale goes back to 1.0), ramp it up slowly over 0.5s to prevent sudden acceleration surges and wheel slippage.

---

## [2026-06-06 13:00:00 -07:00] Iteration 30 Analysis

### 123. Depth-Range-Adaptive Morphological Filtering
* **Context:** `_extract_depth_centroids` uses a fixed 5x5 ellipse kernel for morphological opening and closing to clean noise.
* **The Issue:** Dynamic obstacles (pedestrians) far away appear very small in the depth image. A large 5x5 kernel can completely erase far-away pedestrians (below 1.5–2.0 meters, 5x5 is fine, but at 3.5m a person's foot/leg might only be a few pixels wide). Conversely, a small kernel doesn't clean reflections and noise from nearby specular surfaces.
* **Proposed Enhancement:**
  - Dynamically scale the morphological structuring element kernel size based on depth.
  - For regions of interest or depth frames where closest centroids/hypotheses are far, downscale the kernel to 3x3 to preserve tiny far-off features.
  - For close-range, scale it up to 7x7 to wipe out wide reflection noise.

### 124. IMU-Aided Visual Odometry Ego-Motion Projection
* **Context:** In `velocity_estimator.py`, coordinate projection from local to global frames utilizes EKF-fused pose messages.
* **The Issue:** In high-slip conditions (e.g. pivoting on carpet), wheel odometry lags, causing the estimated global coordinates to slide and creating artificial "ghost velocities" for static features.
* **Proposed Enhancement:**
  - Read high-frequency angular velocity directly from the IMU topic (`/imu/data` `angular_velocity.z`) and integrate it locally over the frame time delta $dt$ to correct the EKF yaw estimate.
  - Fusing raw IMU gyro changes directly into the ego-motion projection loop reduces rotational latency error, stabilizing velocity estimates during quick spins.

### 125. Dynamic Potential Fields for Path Clearance Margins in Narrow Corridor Navigation
* **Context:** In `ab_comparison_test.py`, the side clearance is computed statically, resulting in binary wall collision avoidance.
* **The Issue:** If the corridor is narrow, a binary threshold can cause the robot to oscillate or get stuck if both side sensors declare blockages.
* **Proposed Enhancement:**
  - Implement a dynamic potential field where the repulsive force scales non-linearly: $U_{rep} = \frac{1}{2}\eta (\frac{1}{d} - \frac{1}{d_{max}})^2$.
  - If the corridor narrows below a threshold (e.g., $1.2\text{m}$ total clearance), scale down $d_{max}$ so the robot is willing to squeeze through with a tighter tolerance, rather than pausing or locking up.

### 126. Vectorized Depth Slicing using Bounding Box Coordinates
* **Context:** In `_extract_depth_centroids`, finding valid depth values uses boolean indexing on `cnt_mask == 255` over the entire bounding box slice.
* **The Issue:** Boolean indexing creates a copy of the slice mask and element values, which is slow.
* **Proposed Enhancement:**
  - Instead of drawing the contour mask and applying boolean indexing on all pixels in the bounding box, construct a 1D slice of the depth array by querying a sub-sampled grid of coordinates directly from the contour hull or polygon approximation (`cv2.approxPolyDP`).
  - This retrieves a smaller, representative set of depth values, skipping the full-sized mask drawing and comparison operations.

### 127. WebSocket Frame Compression Bypass for Small Payloads
* **Context:** `server_x3.py` sends telemetry readouts at 20Hz.
* **The Issue:** Even without image bytes, compressing small JSON objects (under 1KB) using compression algorithms like deflate adds CPU overhead without any meaningful savings in network bandwidth.
* **Proposed Enhancement:**
  - Set a size threshold (e.g., 2KB) below which frames are sent as uncompressed text/binary.
  - Only apply compression/deflate if the payload includes image bytes or large map grids, saving valuable CPU cycles on the Jetson Orin.

---

## [2026-06-06 14:00:00 -07:00] Iteration 31 Analysis

### 128. Auto-Calibrating Depth Threshold Offset via Ambient Lighting Reference
* **Context:** `_extract_depth_centroids` uses a fixed grayscale threshold `120` in BGR depth frame fallback mode.
* **The Issue:** Changes in ambient lighting or camera exposure parameters can shift the normalized pixel intensity levels of static background items, causing false contour triggers.
* **Proposed Enhancement:**
  - Compute the average brightness of the background pixels (e.g. upper $10\%$ of the depth frame representing structural ceiling/walls) at startup.
  - Apply a dynamic threshold offset: $\text{THRESH} = \text{default\_thresh} + (\text{mean\_ambient} - \text{baseline\_ambient}) \cdot K$.
  - This keeps contour extraction invariant to background luminance shifts.

### 129. Multi-Scale Temporal Windowing for Motion Feature Engineering
* **Context:** In `velocity_estimator.py`, coordinate features are built using a fixed queue history size of 10 frames ($1.0\text{ s}$).
* **The Issue:** A constant 1.0s history doesn't capture fast-moving dynamic properties (acceleration/deceleration) cleanly, nor does it capture slow, subtle drifts over a longer period.
* **Proposed Enhancement:**
  - Build a multi-scale feature vector consisting of a short-term window ($T_{short}=5$, representing $0.5\text{s}$) to capture quick reaction changes.
  - Pair it with a long-term downsampled window ($T_{long}=15$, downsampled to every other frame representing $3.0\text{s}$) to capture steady gait velocities.
  - This double-temporal resolution improves MLP accuracy for varying speeds.

### 130. Predictive Corridor Yaw Alignment during Bypass Strafing
* **Context:** In `ab_comparison_test.py`, the robot maintains its start yaw `target_yaw` while strafing sideways.
* **The Issue:** In narrow or curved corridors, strafing straight sideways with a fixed yaw can cause the rear or front corners of the robot to clip the side walls due to local layout curvatures.
* **Proposed Enhancement:**
  - Align the robot's yaw command to match the centerline direction of the corridor (extracted from the principal components of the LiDAR scan ranges or map borders).
  - Rather than keeping yaw fixed to the initial global heading, adjust `target_yaw` dynamically to stay parallel to the local hallway contours, maximizing lateral margins during bypass maneuvers.

### 131. Memory-Mapped Shared Ring Buffer for Web Server IPC
* **Context:** `server_x3.py` launches test scripts as subprocesses, and telemetry/sensor reads are passed via asynchronous WebSocket JSON frames.
* **The Issue:** Marshaling large map grids and dynamic estimations through JSON and WebSocket layers introduces serialization/deserialization CPU overhead and latency.
* **Proposed Enhancement:**
  - Create a shared memory segment using Python's `multiprocessing.shared_memory.SharedMemory` or `np.memmap`.
  - Write odometry, lidar scans, and velocity estimates directly to a pre-allocated binary structure.
  - The subprocess can read this memory instantly with zero network/JSON parsing overhead.

### 132. PyTorch JIT Optimization via TensorRT Compilation (Torch-TensorRT)
* **Context:** In `velocity_estimator.py`, PyTorch JIT loads the TorchScript model on CPU.
* **The Issue:** Executing neural network forward passes on the CPU on a Jetson Orin takes valuable resources away from CPU-bound ROS nodes and the WebSocket server.
* **Proposed Enhancement:**
  - Convert the TorchScript MLP to ONNX, then compile it using NVIDIA TensorRT (or use Torch-TensorRT).
  - Run inference directly on the Jetson's GPU or DLA (Deep Learning Accelerator) in FP16 precision.
  - This reduces CPU inference time to sub-millisecond ranges and frees up CPU capacity.

---

## [2026-06-06 15:00:00 -07:00] Iteration 32 Analysis

### 133. Dynamic Scaling of DBSCAN Search Radius (Eps) for LiDAR Points
* **Context:** `server_x3.py` passes the lidar data to `VelocityEstimator` and uses DBSCAN clustering.
* **The Issue:** In narrow doorways or hallways, static walls can cluster together with dynamic pedestrians if the DBSCAN search radius `eps` is too large. If it is too small, a single sparse pedestrian profile at long range will be split into multiple noise clusters, losing tracking continuity.
* **Proposed Enhancement:**
  - Scale the DBSCAN clustering radius `eps` dynamically based on the average density/depth of points in each sector (e.g. `eps(Z) = alpha * Z`).
  - This ensures dense close-range clusters are sharply separated from nearby walls, while maintaining tracking of sparse far-range clusters.

### 134. IMU-Based Visual Centroid Stabilization under Pitch/Roll Vibrations
* **Context:** Pixel coordinates are projected to meters assuming a flat, stable horizontal plane.
* **The Issue:** When the robot moves over uneven ground or starts/stops abruptly, the camera pitches and rolls. This angular camera jitter shifts the image center, creating false coordinate displacements ($dx, dy$) in the features and feeding noise to the MLP.
* **Proposed Enhancement:**
  - Retrieve raw roll and pitch orientation angles from the IMU at the frame timestamp.
  - Apply a 3D rotation correction matrix to the coordinate projection formula: $X_{stabilized} = R_{pitch} R_{roll} X_{local}$.
  - This keeps physical centroids stable against chassis vibrations and sudden acceleration leans.

### 135. Trajectory-Aware Predictive Safety Slowdown Gating
* **Context:** In `ab_comparison_test.py`, the speed scaling decreases maximum velocity if any pedestrian's predicted Time-to-Collision (TTC) is short.
* **The Issue:** Dynamic pedestrians walking parallel to the robot in a narrow hallway can have a small Euclidean distance but zero risk of collision. The robot unnecessarily slows down, hurting its performance score.
* **Proposed Enhancement:**
  - Calculate the lateral overlap (cross-track distance) between the robot's projected corridor and the pedestrian's predicted path.
  - Only trigger speed scaling/pauses if the lateral overlap is less than the safety margin width ($0.45\text{m}$).
  - This allows the robot to drive past parallel-walking pedestrians at full speed.

### 136. Vectorized Frame Downsampling via Slice Offsets
* **Context:** In `_extract_depth_centroids`, downsampling raw depth images is done using bilinear or nearest-neighbor interpolation via `cv2.resize`.
* **The Issue:** Invoking `cv2.resize` on 640x480 float32 arrays on a single CPU thread at 10Hz adds execution latency.
* **Proposed Enhancement:**
  - Downsample the depth array natively in NumPy using basic slicing offsets: `downsampled = raw_depth_frame[::2, ::2]`.
  - This requires no memory copying or library overhead, downsampling the frame in sub-microsecond timescales.

### 137. Pre-Compiled PyTorch Modules via JIT Tracing (Trace-Optimization)
* **Context:** The model loaded is TorchScript, evaluated on dynamic tensors.
* **The Issue:** Dynamic tensor instantiations and model evaluations inside loop-like iterations prevent compilation optimizations.
* **Proposed Enhancement:**
  - Use JIT tracing (`torch.jit.trace`) on a static dummy input tensor of shape $(\text{MAX\_OBSTACLES}, 40)$ at server startup.
  - Run the tracked features directly through this traced module in a single execution path, allowing PyTorch's compiler to optimize kernel fusions and memory layouts.

---

## [2026-06-06 16:00:00 -07:00] Iteration 33 Analysis

### 138. Vectorized Spatial Grid Binning for Lidar Points
* **Context:** In `ab_comparison_test.py`, the lidar scans are processed by iterating through range values to calculate clearance margins.
* **The Issue:** Python loops calculating trigonometry or checking bounds for thousands of lidar beams at 10-20Hz add CPU interpreter overhead.
* **Proposed Enhancement:**
  - Precompute index masks for front, rear, left, and right angular sectors during initialization.
  - Bin raw ranges directly using NumPy masking (e.g., `front_ranges = ranges[front_mask]`), completely avoiding per-frame coordinate calculations and loops.
  - Extract min/mean clearances using fast vector operations to check wall and obstacle bounds instantly.

### 139. Dynamic Camera Gain & Auto-Exposure Control for Shadow Adaptability
* **Context:** Dynamic centroid extraction in `velocity_estimator.py` relies on depth range thresholds.
* **The Issue:** Transitioning from bright open classrooms into dark corridor shadows causes depth return density to fluctuate, leading to contour fragmentation and track dropouts.
* **Proposed Enhancement:**
  - Monitor the average pixel return count and variance of the depth image.
  - Dynamically command camera auto-exposure compensation or apply local contrast enhancement (CLAHE) on the confidence channels.
  - This preserves dynamic shape features and prevents tracking dropout under harsh indoor lighting transitions.

### 140. Multi-Track Kalman Filter Prediction for MLP Window Alignment
* **Context:** The velocity estimator MLP expects sequence inputs representing consecutive historical coordinates sampled at a uniform 10Hz.
* **The Issue:** Sensor occlusions or processor lags can skip frames, producing non-uniform sequence intervals that skew displacement inputs and corrupt velocity predictions.
* **Proposed Enhancement:**
  - Back each active track with a simple 2D constant-velocity Kalman Filter.
  - If a frame detection is missed, use the Kalman state prediction to dead-reckon and append the coordinate to the sequence buffer.
  - This guarantees the MLP receives a clean, evenly spaced 10Hz history queue, increasing prediction stability.

### 141. Active Lateral Centering Potential Field for Corridor Navigation
* **Context:** The software clearance guard in `ab_comparison_test.py` restricts lateral commands if clearances drop below 0.35m.
* **The Issue:** Hard-threshold boundaries can cause the robot to oscillate or scrape the wall when trying to execute heading corrections.
* **Proposed Enhancement:**
  - Implement a continuous potential field centering vector: $v_{y\_corr} = K_p \cdot (clearance_{left} - clearance_{right})$.
  - Blend this corrective force into the lateral command `vy_cmd` during straight segments.
  - This dynamically nudges the robot toward the center of narrow pathways, maintaining a safe spatial buffer on both sides.

---

## [2026-06-07 00:00:00 -07:00] Iteration 34 Analysis

### 142. Gated Recurrent Unit (GRU) Temporal Feature Encoding
* **Context:** The current `velocity_estimator.py` uses a simple Multi-Layer Perceptron (MLP) that receives flattened coordinate displacement sequences (40 features).
* **The Issue:** The MLP treats temporal sequences as independent inputs, ignoring sequential dependencies. Sudden shifts in coordinate measurements or framerate drops degrade velocity predictions.
* **Proposed Enhancement:**
  - Replace the MLP with a Gated Recurrent Unit (GRU) sequence network.
  - Pass the trajectory coordinate displacements sequence sequentially through a single GRU layer to extract latent temporal features before predicting velocity vectors.
  - This naturally accommodates temporal transitions and smooths coordinate predictions.

### 143. Dynamic Point Cloud Downsampling with Adaptive Voxel Grid Gating
* **Context:** The depth processor downsamples frames uniformly or uses standard contours.
* **The Issue:** In narrow pathways, dynamic obstacles are close to the sensor and generate high-density point clusters, while farther obstacles are sparse. Under fixed downsampling, close static objects can merge with dynamic ones, and far dynamic ones are missed.
* **Proposed Enhancement:**
  - Apply an adaptive Voxel Grid Filter where the voxel resolution dynamically scales based on regional depth density.
  - Close-range sectors use a tight voxel grid (e.g. 0.02m) to isolate boundary features, while far-range sectors use larger voxels to suppress noise.
  - This preserves dynamic contours in narrow doorways/corridors without merging them with walls.

### 144. LiDAR Scan Matching for Relative Corridor Slip Estimation
* **Context:** In `ab_comparison_test.py`, the path is tracked using EKF-fused wheel and IMU odometry.
* **The Issue:** In narrow corridors, small slips on mecanum wheels create uncompensated cross-track EKF drift, causing the robot to steer towards side walls.
* **Proposed Enhancement:**
  - Run a fast Iterative Closest Point (ICP) or Correlative Scan Matching (CSM) alignment on successive 2D LiDAR scans to compute a high-precision lateral velocity delta independent of wheel encoder slip.
  - Fuse this LiDAR scan-matching offset directly into the tracking cross-track error $path_y$ computation.
  - This provides direct spatial slide compensation relative to physical corridor boundaries.

### 145. Zero-Copy Memory-Mapped Frame Allocation via Shared Memory IPC
* **Context:** In `server_x3.py`, camera frame images and depth arrays are captured, resized, and encoded in Python before sending.
* **The Issue:** Frequent large array allocations, resizing, and array slicing in Python trigger heavy garbage collection overhead and high CPU use.
* **Proposed Enhancement:**
  - Pre-allocate a fixed size Shared Memory ring buffer segment at startup.
  - Use `np.ndarray(buffer=...)` in the camera capture thread to write frames directly into shared memory with zero memory copy.
  - Resizing and downscaling operations can write back to pre-allocated buffers, reducing Python heap memory allocation and garbage collection cycles to zero.

### 146. Visual-LiDAR Geometric Depth Fusion (Sensor Fusion Gating)
* **Context:** The obstacle detector handles camera estimation and LiDAR scanning separately.
* **The Issue:** Reflections or lens dirt create ghost centroids in the camera, and flat surfaces create false wall edge boundaries in the LiDAR.
* **Proposed Enhancement:**
  - Implement a visual-LiDAR geometric projection gate.
  - Project camera bounding boxes onto the 2D LiDAR plane to filter LiDAR sweeps.
  - A track is only initiated if a dynamic camera bounding box aligns spatially and temporally with a LiDAR range cluster segment. This prevents camera-only reflection triggers and LiDAR-only flat wall detection errors.

---

## [2026-06-07 02:00:00 -07:00] Iteration 35 Analysis

### 147. Scan-Match Corrected Ego-Motion Compensation for Tracker History
* **Context:** In `velocity_estimator.py`, historical coordinates are rotated back using EKF-reported robot pose.
* **The Issue:** Slippage on mecanum wheels introduces uncompensated EKF odometry drift, which distorts the tracked history sequence. This causes the MLP to receive warped trajectories and predict incorrect velocity vectors.
* **Proposed Enhancement:**
  - Expose the scan-match corrected pose (`corrected_x`, `corrected_y`, `corrected_yaw`) from the navigation script to `velocity_estimator.py` (via `server_x3.py`).
  - Use the drift-free scan-matched pose instead of raw EKF pose to transform depth centroids to global coordinates and reconstruct the robot-frame local history window.
  - This eliminates EKF-slippage distortion in the historical inputs, improving the model's accuracy.

### 148. Depth Gradient Watershed Segmentation for Close-Proximity Obstacle Separation
* **Context:** Obstacle centroids are extracted from binary depth masks inside `velocity_estimator.py` using `cv2.findContours`.
* **The Issue:** When a pedestrian walks close to doorways, doors, or corridor walls, the depth values of the wall and the human overlap, causing standard contour detection to merge them into a single blob and bias the centroid.
* **Proposed Enhancement:**
  - Compute spatial depth gradients using Sobel or Scharr operators on the raw float32 depth frame to detect surface boundaries.
  - Apply marker-controlled watershed segmentation on depth minima to isolate close-proximity obstacles from wall surfaces.
  - This ensures dynamic objects are correctly isolated and centered even when driving through tight gaps.

### 149. Yaw-Aware Dynamic Repulsion & Clearance-Based Rotation Center Shifting
* **Context:** The potential field repulsion `vy_rep` in `ab_comparison_test.py` is calculated and applied to lateral command speeds only during straight driving segments.
* **The Issue:** During turnaround states (`ROTATE_180` and `ROTATE_HOME`), the robot's square footprint corners sweep a wider radius than its circular clearance, risking side-swipe collisions when rotating in narrow corridors.
* **Proposed Enhancement:**
  - Project the robot's physical chassis corner vertices into the current LiDAR frame during rotation states.
  - If any corner is predicted to violate the 0.30m wall clearance threshold during yaw changes, calculate a dynamic lateral correction force.
  - Command a tiny corrective lateral mecanum strafe velocity along with the rotational yaw velocity, shifting the rotation center dynamically to keep the chassis centered in the corridor during turnaround.

### 150. Fast 1D Angular-Index Search for Lidar Scan Matching
* **Context:** The scan matcher in `ab_comparison_test.py` runs ICP iterations to align current and previous laser point clouds.
* **The Issue:** Computing the full 2D distance matrix ($M \times N$) of size $360 \times 360$ via broadcasting consumes excessive CPU cycles and memory bandwidth at 20Hz on the Jetson.
* **Proposed Enhancement:**
  - Convert incoming scan points to polar coordinates (radius and angle) relative to the sensor.
  - For each point in the current scan, perform a direct $O(1)$ angular index lookup in the previous scan array to find the closest angular range measurement.
  - This replaces the $O(M \times N)$ spatial search with an $O(M)$ lookup, eliminating large distance matrix allocations and reducing scan alignment CPU overhead to negligible levels.

### 151. Auto-Calibrating Homography & Projection Alignment via YOLO Centroid Feedback
* **Context:** Depth centroids are projected into 2D image space in `velocity_estimator.py` to check for intersection with YOLO bounding boxes.
* **The Issue:** Manufacturing mount tolerances, lens distortion, or physical camera pitch/roll offsets can misalign the projected depth points and YOLO visual boxes, causing valid centroids to be gated out.
* **Proposed Enhancement:**
  - Implement a recursive least squares (RLS) estimator that tracks the displacement between projected depth coordinates and the centers of static YOLO person detections.
  - Adaptively tune the projection matrix parameters (focal lengths, pitch/roll camera offsets) to minimize visual-to-depth alignment error.
  - This self-calibrating loop ensures robust gating even under physical sensor alignment changes or camera vibration.

---

## [2026-06-07 03:00:00 -07:00] Iteration 36 Analysis

### 152. Physically Constrained MLP Output Gating (Kinematic Acceleration Bounding)
* **Context:** The MLP model in `velocity_estimator.py` predicts raw $(vx, vy)$ pedestrian velocity values frame-by-frame.
* **The Issue:** Measurement noise or sequence drops can lead to spike predictions (e.g. predicting a person accelerating at $10\text{ m/s}^2$), causing erratic speed scaling and vehicle jerk.
* **Proposed Enhancement:**
  - Implement a physical kinematics filter on the outputs of the velocity estimator.
  - Clamp predicted velocities between consecutive frames so that the implied acceleration does not exceed maximum human running/walking limits (e.g. $\pm 3.0\text{ m/s}^2$).
  - Enforces smooth, physically feasible trajectory predictions without requiring model retraining.

### 153. LiDAR Range-Cluster Guided Depth ROI Extraction
* **Context:** `velocity_estimator.py` downsamples and processes the entire raw depth frame to extract obstacle contours.
* **The Issue:** Scanning the full $480 \times 640$ matrix for contours is CPU-intensive and prone to detecting background features (like door frames or tables) in narrow spaces.
* **Proposed Enhancement:**
  - Intersect 2D LiDAR range cluster coordinates with the camera's visual field of view (FOV).
  - Crop small bounding boxes (Regions of Interest) around these coordinates in the depth frame.
  - Run the dynamic downsampling and contour extraction only within these cropped ROIs rather than the full image. This isolates dynamic targets and reduces depth processing CPU overhead.

### 154. Predictive Trajectory-Intersector Bypassing (TTC-Guided Path Planning)
* **Context:** In `ab_comparison_test.py`, the robot chooses a lateral bypass direction (left/right) based on current side clearances.
* **The Issue:** If a pedestrian is walking diagonally across the corridor, the robot might steer into the pedestrian's future path, causing a collision or forcing a hard stop.
* **Proposed Enhancement:**
  - Calculate a Time-to-Collision (TTC) vector by projecting the robot's path and the pedestrian's velocity vector ($vx, vy$).
  - If a collision is predicted, calculate the intersection point.
  - Set the lateral bypass target offset to steer the robot behind the pedestrian's travel vector (clearing their path) rather than cutting in front of them.

### 155. Vectorized LiDAR Point Clustering via Range Gradient Masking
* **Context:** Clustering LiDAR points for obstacle tracking often requires running spatial density algorithms.
* **The Issue:** Standard spatial clustering algorithms (e.g. DBScan) add high computational overhead when run at 20Hz on the Jetson.
* **Proposed Enhancement:**
  - Implement a vectorized 1D clustering routine directly on sorted polar range arrays using NumPy.
  - Check the range differences between adjacent angular beams: `mask = np.abs(ranges[1:] - ranges[:-1]) < epsilon`.
  - Slice and group connected segments directly using index arrays. This clusters the entire scan in $O(N)$ time with minimal CPU footprint.

### 156. Real-Time Control Jitter and Energy Efficiency Metrics Logger
* **Context:** `ab_comparison_test.py` records trajectories, obstacle speeds, and clearances to a CSV log.
* **The Issue:** Baseline logs do not capture key operational metrics such as actuator strain, trajectory deviation, or timing jitter, which are vital for quantitative A/B comparisons.
* **Proposed Enhancement:**
  - Append control command jitter (variance of the 20Hz loop step intervals), path centering deviation (integral of squared cross-track error: $\int path_y^2 dt$), and mechanical energy expenditure (integral of command velocities: $\int (vx^2 + vy^2 + \omega^2) dt$) to the CSV logger.
  - This provides concrete quantitative indicators of smoothness, battery consumption, and path performance between modes.

---

## [2026-06-07 04:00:00 -07:00] Iteration 37 Analysis

### 157. Adaptively Adjusted Online Feature Normalization Gating
* **Context:** Features fed to the MLP model in `velocity_estimator.py` are normalized using fixed mean and scale parameters loaded from `scaler_params.json` at startup.
* **The Issue:** When operating in rooms or corridors with layout dimensions that differ significantly from the training environment, input features can drift out of the model's training distribution, degrading prediction accuracy.
* **Proposed Enhancement:**
  - Track running online mean and variance of track coordinate displacements.
  - If online statistics deviate significantly from training parameters, apply an adaptive scaling layer that maps coordinates back toward the model's expected input bounds.
  - Improves model generalization under diverse spatial settings without retraining.

### 158. LiDAR-Depth Dynamic Extrinsics Auto-Tuning via Ground-Plane Invariance
* **Context:** Visual-LiDAR gating projects 3D depth centroids onto camera coordinate grids using static translation/rotation parameters.
* **The Issue:** Acceleration/deceleration on mecanum wheels causes chassis pitch/roll tilts. This distorts the spatial coordinates of depth centroids, leading to camera-LiDAR projection misalignment.
* **Proposed Enhancement:**
  - Query real-time IMU pitch/roll values or fit a ground-plane model to raw LiDAR scans.
  - Calculate a dynamic rotation adjustment matrix to apply real-time pitch/roll tilt corrections to the projection matrices.
  - Maintains precise camera-to-LiDAR spatial alignment during dynamic motion.

### 159. RANSAC Line-Fitting for Corridor Wall Boundary Identification
* **Context:** In `ab_comparison_test.py`, potential field wall clearances are computed by finding the local range minimum in the side sectors.
* **The Issue:** Isolated obstacles (like chair legs or trash cans) in the side sectors are mistaken for corridor walls, causing the robot to center itself incorrectly or wobble.
* **Proposed Enhancement:**
  - Run a fast RANSAC line-fitting algorithm on the left/right sector LiDAR scans to isolate the continuous wall planes.
  - Compute repulsion forces and centering commands relative to these fitted wall lines instead of raw minimum clearances.
  - Stabilizes path centering and prevents wobbling around isolated side obstacles.

### 160. Fast-Path Gating for Empty Track States
* **Context:** `velocity_estimator.py` runs its full depth thresholding, tracking association, and PyTorch inference pipeline at 10Hz.
* **The Issue:** Running PyTorch model calls and contour calculations when the scene is completely clear wastes GPU/CPU resources on the Jetson.
* **Proposed Enhancement:**
  - Place a fast-path gate at the beginning of the `_inference_loop`.
  - If raw depth frames show no pixel clusters above threshold, immediately skip the tracker update and PyTorch model calls.
  - This drops estimation loop CPU overhead to near-zero when driving in clear corridors.

### 161. Bounding Box Temporal Prediction Gating for Slow YOLO Framerates
* **Context:** Image projection gates project camera person bounding boxes to filter depth centroids.
* **The Issue:** Querying YOLO on every frame adds high processing overhead and decreases frame rates.
* **Proposed Enhancement:**
  - Track and project person bounding boxes forward in time using their estimated velocity.
  - Use these predicted bounding boxes to gate depth centroids when new YOLO frames are not yet available.
  - Enables running the projection gate at a fast 10Hz rate even when YOLO detections are throttled to a lower rate (e.g. 2Hz) to save resources.

---

## [2026-06-07 05:00:00 -07:00] Iteration 38 Analysis

### 162. Ego-Velocity-Weighted Feature Regularization
* **Context:** The MLP model in `velocity_estimator.py` receives a flattened sequence of 40 coordinate displacement features.
* **The Issue:** During high-acceleration motions or sharp rotations, tracking latency and EKF noise create coordinate displacement artifacts, causing the model to predict false velocities for static surroundings.
* **Proposed Enhancement:**
  - Feed the robot's active linear and angular base velocities $(v_{x\_base}, v_{y\_base}, \omega_{z\_base})$ as additional inputs to the model.
  - This expands the input layer to 43 features, allowing the model to learn base kinematic stress correlations and correct visual flow distortion.
  - Reduces false velocity predictions during high-stress rotational or acceleration maneuvers.

### 163. Bilateral Depth Filtering for Doorway Contour Preservation
* **Context:** Voxel grid downsampling decimates pixel density to clean noise and accelerate contour calculations in `velocity_estimator.py`.
* **The Issue:** Decimation smooths out high-frequency depth transitions, blurring doorway edges and causing door frames to merge with pedestrian centroids.
* **Proposed Enhancement:**
  - Apply a fast 1D bilateral filter on depth rows prior to decimation.
  - The bilateral filter smooths out range noise while preserving sharp depth boundary edges.
  - Prevents tracking dropouts and target merging in doorway openings and narrow corridor transitions.

### 164. Corridor Intersection Detection & Speed Profiling via Angular LiDAR Entropy
* **Context:** `ab_comparison_test.py` drives the robot at speeds scaled strictly by obstacles directly in its path.
* **The Issue:** When approaching blind corridor intersections, the robot drives at full speed because it cannot see around corners, risking collisions with emerging pedestrians.
* **Proposed Enhancement:**
  - Monitor the Shannon entropy or variance of range returns in the side $90^\circ$ sectors.
  - A sudden increase in entropy or range indicates a corridor intersection opening.
  - When an intersection is detected, scale down the forward speed profile to safely anticipate obstacles turning the corner.

### 165. Multi-Threaded Frame-Fetch Gating (Producer-Consumer Queue)
* **Context:** `velocity_estimator.py` queries `get_depth_frame()` and `get_raw_depth_frame()` synchronously at 10Hz.
* **The Issue:** If the topic bridge blocks waiting for DDS serialization, the estimation loop stalls, introducing jitter into control command updates.
* **Proposed Enhancement:**
  - Run frame-grabbing in a separate producer thread that writes to a thread-safe, single-element buffer.
  - The inference loop fetches frames non-blockingly from the buffer. If no new frame is available, it skips the step, preventing DDS delays from stalling the control loop.

### 166. Dynamic Depth Segment Size Thresholding based on Range
* **Context:** Centroid extraction filters out contour blobs smaller than a fixed `MIN_BLOB_AREA` (500 pixels).
* **The Issue:** As target distance increases, the projected pixel area shrinks, causing far-away pedestrians to be filtered out, while near static table legs are tracked.
* **Proposed Enhancement:**
  - Scale the minimum pixel area threshold dynamically based on target depth $Z$: `MinArea(Z) = BaseArea * (Z_ref / Z)^2`.
  - Ensures far-away pedestrians (e.g. at 3.5m) are successfully tracked, while suppressing nearby static noise.

---

## [2026-06-07 06:00:00 -07:00] Iteration 39 Analysis

### 167. Relative Acceleration Input Features for Intercept Stability
* **Context:** Sequence trajectories in `velocity_estimator.py` represent historical $(x, y)$ positions and approximate velocities.
* **The Issue:** Sudden velocity or heading changes of a pedestrian are not immediately represented in position-velocity histories, delaying reactive bypass planning.
* **Proposed Enhancement:**
  - Calculate relative acceleration (the rate of change of relative velocity between sequence frames) and append them directly to the feature vector.
  - This provides the neural network with explicit trajectory curvature features, improving prediction speed when pedestrians turn or brake.

### 168. Reflectivity Filtering for Specular Ground and Metallic Surface Exclusions
* **Context:** Navigation and tracking algorithms segment obstacles based on distance returns from the depth camera and LiDAR.
* **The Issue:** Highly reflective surfaces (e.g., polished hallway floors, glass partitions, or metallic doors) create mirror reflections, generating ghost depth centroids and fake LiDAR readings.
* **Proposed Enhancement:**
  - Check the LiDAR range return intensity (reflectivity values) provided by the sensor driver.
  - Specular ground and glass surfaces exhibit unique intensity profiles. By filtering out points that violate diffuse surface intensity thresholds, we can exclude ghost obstacles.

### 169. Dynamic Corridor Angle Yaw-Alignment Control
* **Context:** `ab_comparison_test.py` aligns the robot's heading target to the initial yaw recorded at start (`self.target_yaw = self.start_yaw`).
* **The Issue:** If the robot starts slightly rotated relative to the corridor walls, it drives diagonally, triggering potential field wall repulsions and steering adjustments.
* **Proposed Enhancement:**
  - Extract the angle of the corridor walls using the fitted RANSAC side lines.
  - Dynamically adjust `self.target_yaw` to align exactly parallel to the centerline of the hallway.
  - Ensures the robot drives parallel to the corridor, reducing active steering corrections.

### 170. Direct Tensor Sharing via CUDA IPC (Unified GPU Memory Pipeline)
* **Context:** Pre-allocated shared memory stores frames in host CPU RAM, which requires a host-to-device memory copy when copying features into PyTorch tensors.
* **The Issue:** Copying data from host (CPU) to device (GPU) adds memory latency and blocks Python execution threads at high frame rates.
* **Proposed Enhancement:**
  - Allocate PyTorch tensors directly on the GPU utilizing Unified Memory or CUDA IPC.
  - Let the camera capture thread write frame data directly to GPU memory, bypassing CPU host allocation.
  - Eliminates CPU-to-GPU data copies, reducing inference latency.

### 171. Vectorized LiDAR-based Yaw-Drift Corrector
* **Context:** ICP scan matching is run on full point clouds to compute lateral and yaw drift corrections.
* **The Issue:** Aligning entire point clouds is computationally expensive and wastes CPU cycles if only yaw drift needs correction.
* **Proposed Enhancement:**
  - Find the angular index $\theta_{min}$ in the side LiDAR sectors where the range measurement is absolute minimum.
  - In a straight corridor, this minimum range points directly perpendicular to the side wall. The angular deviation of $\theta_{min}$ from $90^\circ$ gives the yaw drift directly.
  - Corrects EKF yaw drift via an $O(N)$ index lookup, bypassing ICP execution.

---

## [2026-06-07 07:00:00 -07:00] Iteration 40 Analysis

### 172. Multi-Modal Model Fusion (LiDAR-Feature Combined MLP)
* **Context:** The MLP model in `velocity_estimator.py` uses sequence windows of depth-centroid coordinates.
* **The Issue:** Centroid coordinates are susceptible to depth noise and occlusions, leading to tracking errors.
* **Proposed Enhancement:**
  - Extract physical shape features of the pedestrian from the LiDAR scans (e.g. cluster width, boundary curvature, number of returned beams).
  - Append these structural features as static inputs to the model alongside depth tracking coordinate history.
  - Fusing structural LiDAR geometry with depth history provides richer inputs, improving prediction accuracy.

### 173. Self-Adapting Contrast Enhancement (Local CLAHE) for Shadow Exclusions
* **Context:** Depth segment contours are extracted by thresholding raw depth frames.
* **The Issue:** Doorways and corners in narrow corridors often contain dark shadow regions, where IR light returns are weak or noisy, causing contour fragmentation.
* **Proposed Enhancement:**
  - Apply Contrast Limited Adaptive Histogram Equalization (CLAHE) on the depth confidence or IR channel before thresholding.
  - This adaptive local contrast enhancement preserves the outlines of obstacles in dark shadows.
  - Prevents target tracking drops in poorly lit corners.

### 174. Predictive Deceleration Profiling for Dynamic Obstacle Crossings
* **Context:** Speed scaling in `ab_comparison_test.py` is reactive: it slows down the robot when obstacles enter the corridor envelope.
* **The Issue:** If a pedestrian is walking diagonally across the corridor, the robot drives at full speed until they cross, leading to abrupt braking.
* **Proposed Enhancement:**
  - Estimate the pedestrian's path crossing time and the robot's arrival time at the intersection point.
  - If a collision is predicted, scale down command speed beforehand (proactive speed profiling).
  - Avoids abrupt emergency stops and provides smooth velocity transitions.

### 175. Vectorized 2D Rotation using NumPy Multi-Dimensional Matrix Dot Products
* **Context:** Historical global coordinates are rotated back to local coordinates in `velocity_estimator.py` using Python loops.
* **The Issue:** Running per-frame coordinate rotations in Python loops for all active tracks adds CPU overhead.
* **Proposed Enhancement:**
  - Stack all history coordinates into a shape $(T, 2)$ matrix.
  - Construct a batch 2D rotation matrix and perform a single matrix multiplication using a vectorized NumPy dot product.
  - Bypasses Python loop overhead and leverages BLAS acceleration.

### 176. Non-Blocking Asynchronous Telemetry Logger
* **Context:** Log rows are recorded to the CSV file or disk in the main thread of the comparison script.
* **The Issue:** Disk I/O operations can block the main execution loop, creating control updates jitter.
* **Proposed Enhancement:**
  - Implement a queue-based logger that pushes telemetry rows to a thread-safe queue.
  - Let a background worker thread read the queue and execute disk writes asynchronously.
  - Prevents disk write latency from introducing jitter into the control loop.

---

## [2026-06-07 08:00:00 -07:00] Iteration 41 Analysis

### 177. Multi-Scale Temporal Feature Pooling (Pyramid Temporal History Window)
* **Context:** Track history buffers in `velocity_estimator.py` are sampled at a fixed 10Hz rate with a window size of 10 (representing 1.0s history).
* **The Issue:** A fixed history window is too short to accurately estimate slow-moving pedestrians (whose coordinates change slowly relative to sensor noise) and too long for fast-accelerating targets.
* **Proposed Enhancement:**
  - Sample the track history at two scales: a short-term window (e.g. last 5 frames at 10Hz, covering 0.5s) and a long-term window (e.g. 5 frames sampled at 5Hz using every second frame, covering 2.0s).
  - Feed both timescale windows into the MLP input layers.
  - This multi-scale temporal pooling allows the model to capture both slow drifts and fast changes.

### 178. LiDAR Intensity-based Dynamic Floor Segment Filter
* **Context:** Ramps, doorways, or vehicle pitch tilts can cause the ground floor to project into LiDAR sectors, creating false obstacle detections.
* **The Issue:** Floor contacts generate range readings, which are mistaken for walls or pedestrians, triggering false collision checks.
* **Proposed Enhancement:**
  - Use the returned reflectivity/intensity values of LiDAR sweeps.
  - Ground surfaces scatters light differently than vertical human clothing or vertical walls. By applying intensity thresholds on the incoming ranges, exclude ground-plane returns.
  - Prevents false doorway and ramp navigation stops.

### 179. Proactive Deceleration Profiling for Lateral Wall Clearances
* **Context:** In `ab_comparison_test.py`, potential field wall centering is computed.
* **The Issue:** In tight, narrow corridor sectors, executing lateral corrections at full speed can cause the robot to oscillate or drift too close to the walls before centering.
* **Proposed Enhancement:**
  - Scale down the maximum allowed forward velocity proportionally to the current lateral clearances.
  - `max_linear_speed = BaseMaxSpeed * min(1.0, wall_clearance / safety_threshold)`.
  - Automatically slows down the robot in tight sections, giving the lateral controller more time to correct slip.

### 180. Vectorized LiDAR Corner/Vertex Extraction via RDP Line Simplification
* **Context:** Collision checking and potential field repulsion use raw arrays of 360 LiDAR range measurements.
* **The Issue:** Performing distance checks, wall projections, and blockage checks on all 360 beams on every frame is CPU-intensive.
* **Proposed Enhancement:**
  - Run the vectorized Ramer-Douglas-Peucker (RDP) algorithm on the LiDAR coordinate array.
  - Simplify the 360 points into a small set of 4 to 6 vertices representing the true walls and corners.
  - Execute collision checking and wall repulsion target calculations on the simplified vertices, reducing computational overhead.

### 181. Double-Buffered Shared Memory IPC
* **Context:** Pre-allocated shared memory blocks are written by the capture thread and read by the server/visualization threads.
* **The Issue:** Simultaneous read/write access to shared memory buffers can create screen tearing or partial frame updates.
* **Proposed Enhancement:**
  - Allocate double-buffered shared memory blocks (`shm_bgr_0` and `shm_bgr_1`).
  - Maintain a shared control byte indicating the index of the latest completed frame.
  - The consumer reads the latest completed index and maps its array view to that buffer, ensuring lock-free, race-free IPC.

---

## [2026-06-07 09:00:00 -07:00] Iteration 42 Analysis

### 182. Track-Frame Heading Normalization for MLP Generalization
* **Context:** History coordinates in `velocity_estimator.py` are normalized relative to the robot's local frame orientation before being flattened.
* **The Issue:** When a pedestrian changes direction or the robot rotates, absolute coordinates fluctuate within the history window, degrading model inference accuracy.
* **Proposed Enhancement:**
  - Apply Principal Component Analysis (PCA) on the track history coordinates to determine the primary axis of motion.
  - Rotate the entire coordinate window to align the motion vector along the local $x$-axis.
  - The MLP learns to predict speed relative to the path of motion rather than absolute coordinate directions, improving generalization.

### 183. K-Means guided Active Depth Cropping ROI for Crowded Scenarios
* **Context:** Bounding box regions are cropped around projected targets in `velocity_estimator.py` to isolate targets.
* **The Issue:** In crowded doorways, multiple targets generate overlapping bounding boxes, causing contour detection to merge separate pedestrian blobs.
* **Proposed Enhancement:**
  - Apply a fast, vectorized 1D K-means clustering on raw depth rows (where $K$ is the number of active LiDAR clusters).
  - Use these distinct depth layers to create separate ROI depth masks for each target.
  - Keeps close-proximity targets isolated from one another during contour processing.

### 184. Corridor Cornering Clearance Compensation via Sweep Envelope Expansion
* **Context:** In `ab_comparison_test.py`, the wall clearances are computed statically based on minimum LiDAR ranges.
* **The Issue:** During turnaround turns, the robot's corners sweep a larger area, and wheel slide can cause the robot to drift too close to the side walls.
* **Proposed Enhancement:**
  - Expand the robot's clearance safety bounds proportionally to the base angular velocity $\omega_z$.
  - Construct a dynamic sweep footprint envelope that accounts for rotation radius and slippage.
  - Command counter-lateral mecanum velocities if this sweep envelope intersects wall boundaries during turns.

### 185. Vectorized LiDAR Scan Compaction via Adaptive Decimation
* **Context:** Potential field and scan-matching algorithms process the full array of 360 LiDAR range measurements on every frame.
* **The Issue:** Processing all 360 points at 20Hz creates redundant computational overhead when scanning flat corridor walls.
* **Proposed Enhancement:**
  - Decimate the 360 rays down to a lower resolution (e.g. 90 rays) in flat sectors where range gradients are small.
  - Maintain full resolution in sectors where range changes are steep (e.g., around obstacles or corners).
  - Reduces the number of points processed by potential field and ICP controllers, improving efficiency.

### 186. Temporal Tracking Gate Hysteresis via Multi-Frame Bounding Box Matching
* **Context:** Visual-LiDAR gating requires a depth centroid to intersect a YOLO camera person bounding box on every frame.
* **The Issue:** If the camera frame drops or the target is briefly occluded, the gate fails and the target tracking is dropped immediately.
* **Proposed Enhancement:**
  - Implement a multi-frame tracking gate hysteresis.
  - If a track was confirmed in previous frames, continue to gate it using the last known bounding box (expanded by a temporal search margin) for up to 3 frames of camera occlusion.
  - Prevents track dropouts due to visual frame drops or brief occlusions.

---

## [2026-06-08] Automated Analysis Batch — Ideas 187 to 198

### Idea 187: Precomputed LiDAR Angle Array Cache
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In both `_scan_cb` and `_update_bypass_offset`, the angle for each beam is recomputed as `normalize_angle(angle_min + i * angle_increment + math.pi)` inside a Python for-loop on every single callback. Caching this entire angle vector as a NumPy float32 array whenever `last_scan_angle_min` or `last_scan_angle_increment` changes would allow all sector masks (front, left, right, rear) to be computed in a single vectorized `np.abs` or boolean call, eliminating the per-beam Python loop overhead and enabling 5–10× faster scan processing.

### Idea 188: LaserScan Range Conversion to NumPy Array
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `_scan_cb` converts `msg.ranges` to a Python list via `list(msg.ranges)` on every callback. Every downstream use in `_update_bypass_offset` and the ICP matcher then iterates this list element-by-element in Python for-loops. Converting once to a `np.array(msg.ranges, dtype=np.float32)` enables `np.isnan` filtering, range thresholding, and polar-to-Cartesian conversion to run as single vectorized NumPy calls, yielding roughly a 10× speedup on the inner-loop LiDAR processing that runs at 20 Hz.

### Idea 189: Adaptive ICP Convergence Criterion with Early Exit
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** `_align_scans_icp` always runs exactly 3 iterations regardless of whether the correction has already converged. When the robot is nearly stationary, ICP converges fully in one iteration (corrections < 0.1 mm), yet iterations 2 and 3 still execute and compute the full O(M×N) distance matrix. Adding a convergence check `if abs(dx_corr) + abs(dy_corr) + abs(dtheta_corr) < 1e-4: break` after each iteration saves 1–2 full distance-matrix computations per scan on calm segments, meaningfully reducing CPU load at 20 Hz.

### Idea 190: ICP Scan Match Inlier Quality Gate
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** `_align_scans_icp` applies its computed pose correction unconditionally, regardless of how many point pairs survived the `valid = min_dists2 < 0.0625` inlier filter. In degenerate scenes (open doorways or sparse scans after a large motion), very few points match and the resulting alignment is essentially noise. Adding a gate — if `np.sum(valid) < 0.20 * len(Q_trans)`, fall back to raw odometry for that step — prevents a single bad scan pair from injecting a large spurious drift correction into `corrected_x/y/yaw`.

### Idea 191: Speed-Adaptive Forward LiDAR Detection Range
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** The front blockage check in `_update_bypass_offset` uses a fixed `x_fwd < 0.75m` threshold regardless of the robot's current commanded speed. At `MAX_LINEAR_SPEED = 0.20 m/s` with a `KP_DIST = 0.6` controller, the actual stopping distance can exceed 0.75 m. Scaling the forward detection window dynamically as `look_ahead = max(0.75, last_vx_cmd / KP_DIST + 0.30)` provides proportionally earlier obstacle detection at higher speeds, allowing the speed-scaling logic to begin braking before the robot is already inside the unsafe zone.

### Idea 192: Twist Message Object Pre-Allocation in Control Loop
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `_control_loop` creates a new `Twist()` ROS message object on every 20 Hz iteration via `twist = Twist()`, even during the three settle states that publish only zero-velocity commands. This allocates and zeros a ROS2 C++-backed Python binding 20 times per second. Pre-allocating `self._zero_twist = Twist()` and `self._cmd_twist = Twist()` at `__init__` time and resetting only the relevant fields in-place removes all hot-path message allocations, which reduces minor GIL pressure and object churn.

### Idea 193: Asymmetric Lateral Acceleration and Deceleration Rate Limiting
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** The lateral rate limiter in `DRIVE_TO_B` and `DRIVE_TO_A` applies a symmetric `MAX_LATERAL_ACCEL = 0.5 m/s²` for both increasing and decreasing `vy_cmd`. When the robot is strafing toward a wall and needs to stop lateral motion urgently (e.g. `vy_rep` flips sign), the same gentle ramp slows the correction. Implementing asymmetric limits — a higher deceleration cap (e.g. 1.5 m/s²) when `vy_cmd` moves against the APF repulsion direction, and the existing 0.5 m/s² for acceleration — enables rapid lateral stopping near walls while keeping smooth, slip-free bypass initiation.

### Idea 194: ICP Correction Delta Bound Clamping for Long-Run Stability
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** `_scan_cb` applies ICP-computed displacements to `corrected_x/y/yaw` without any per-step sanity check. In degenerate environments (open corridors with few features), ICP can produce large spurious corrections diverging far from the raw odometry delta. Adding a per-step rejection gate — if `abs(dx_match - dx_odom_local) > 0.05m` or `abs(dtheta_match - dtheta) > 0.05 rad`, fall back to raw odometry — prevents a single bad alignment from permanently corrupting the scan-matched reference pose that the control loop uses as its ground truth.

### Idea 195: MLP Prediction Variance-Based Confidence Weighting for TTC Scaling
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** `_get_speed_scaling()` treats all MLP velocity estimates equally regardless of their temporal stability. For tracks with noisy inputs (partially occluded pedestrians, split contours), the `speed` output fluctuates widely frame-to-frame, causing erratic braking from transient spikes. Maintaining an exponential variance estimate per track (`var = alpha * (speed - mean)**2 + (1-alpha)*prev_var`) and weighting the TTC contribution as `effective_speed = speed * exp(-k * var)` dampens noisy tracks without disabling them, preserving responsiveness to stable, well-tracked obstacles.

### Idea 196: Dynamic Track Association Radius Based on Last Estimated Speed
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** `ObstacleTracker.update()` uses a fixed `max_dist = 0.8m` nearest-neighbor gate for all tracks. This is simultaneously too permissive for slow-moving or stationary objects (where 0.8 m allows a nearby moving pedestrian to steal the ID) and potentially too tight for fast tracks when a frame is dropped (0.2 s gap × 1.2 m/s = 0.24 m, fine, but at 1.5 m/s it approaches the limit). Scaling the threshold as `match_radius = max(0.3, min(0.8, last_track_speed * dt * 2.0 + 0.15))` tightens the gate for stationary tracks and relaxes it for fast ones, reducing identity-switch errors.

### Idea 197: EMA-Smoothed Wall Clearance Values for APF Jitter Prevention
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `wall_left_clearance` and `wall_right_clearance` are set to the raw minimum range value observed in their sectors on each 20 Hz tick. A single noisy LiDAR beam or specular reflection can spike one clearance to a falsely low value for a single frame, causing the APF repulsion force to jerk the robot's lateral command suddenly. Adding a two-element EMA (`smoothed = 0.7 * new_val + 0.3 * prev_val`) to both clearance values before the APF force calculation in `_update_bypass_offset` eliminates one-frame spike artifacts while keeping 70% response bandwidth — no structural changes to the APF math required.

### Idea 198: ICP Point Cloud Uniform Subsampling for CPU Efficiency
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_align_scans_icp` operates on all valid scan points (potentially 200–360 points per cloud), computing an O(M×N) pairwise distance matrix in each of its 3 iterations. For a 360-point scan, the inner distance matrix is 360×360 = 129,600 elements per iteration. Uniformly subsampling both clouds to at most 80 points (`pts = pts[::max(1, len(pts)//80)]`) reduces this to 80×80 = 6,400 elements — a 20× reduction — while preserving alignment accuracy since indoor corridor walls are planar features fully characterized by sparse samples. The fix is a single one-line stride slice requiring no other code changes.

## [2026-06-08] Automated Analysis Batch — Ideas 199 to 208

### Idea 199: Repeat-Mode `prev_scan_points` Reset to Prevent Cross-Run ICP Corruption
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** The `ROTATE_HOME` repeat-mode reset block (~line 1194) clears `corrected_x/y/yaw` and `prev_odom_pose` for a fresh run, but `self.prev_scan_points` retains the robot's final backward-facing scan from the previous run. The first `_scan_cb` of the new run ICP-aligns a fresh forward-facing scan against this stale backward reference, injecting large spurious `dx`/`dtheta` corrections into `corrected_x/y/yaw` before the robot has moved. Fix: add `self.prev_scan_points = []` alongside the existing `prev_odom_pose` reset — a one-line change that eliminates cross-run ICP corruption entirely.

### Idea 200: Non-Blocking Recovery Backing via Timer-Based State
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** The recovery backing maneuver in `DRIVE_TO_B` and `DRIVE_TO_A` (~lines 945–947 and 1136–1139) calls `time.sleep(0.1)` 15 times inside the ROS2 timer callback, blocking the executor thread for 1.5 seconds. During this block, `_odom_cb`, `_scan_cb`, and ICP drift correction are all starved — precisely when backward motion makes LiDAR safety checks most critical. Replace with an `_is_backing` boolean flag and `_backing_start` timestamp checked in the normal 20 Hz control loop path, allowing all sensor callbacks to continue processing during recovery.

### Idea 201: `visible_count` Decay on Unmatched Track Frames in `ObstacleTracker`
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** `ObstacleTracker.update()` (line ~59) increments `visible_count` on a match but never decrements it on a miss. Once a track passes the `visible_count >= 3` inference gate, it remains permanently eligible through arbitrarily long occlusion gaps, generating ghost-track TTC contributions from stale observations. Add `track['visible_count'] = max(0, track['visible_count'] - 1)` in the existing track-aging loop so eligibility degrades on missed frames and requires re-establishment after a sustained occlusion.

### Idea 202: Division Pre-Inversion for Faster Feature Normalization
**Area:** Script Performance
**Investment:** Extremely Low
**Rationale:** `_inference_loop` computes `(features_batch - self.scaler_X_mean) / self.scaler_X_scale` on an (N, 40) float32 matrix at 10 Hz. On ARM NEON (Jetson Orin Nano), a vectorized float32 division is approximately 2× slower than multiplication. Pre-compute `self.scaler_X_inv_scale = (1.0 / self.scaler_X_scale).astype(np.float32)` and analogously for `scaler_y_scale` in `_load_model()`, then replace the per-inference division with `(features_batch - self.scaler_X_mean) * self.scaler_X_inv_scale` — no change to numerical output, measurable reduction in normalization wall time.

### Idea 203: Multi-Frame Confirmation Counter for `_front_is_continuous_wall`
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `_front_is_continuous_wall` is set from a single LiDAR scan in `_update_bypass_offset`. A single frame where forward beams lack edge discontinuities (e.g., a smooth doorframe or scan glitch) can incorrectly suppress camera-based bypass initiation or trigger the static-wall early stop for one control tick. Add a `_wall_confirm_count` integer that increments when the per-frame result is True and resets to zero when False; expose `_front_is_continuous_wall = True` externally only when `_wall_confirm_count >= 2`, providing 100 ms of hysteresis before any downstream action is triggered.

### Idea 204: Per-Track Distance Gate Before MLP Inference to Skip Far Targets
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** All tracks with `visible_count >= 3` and non-zero kinematic displacement are batched for MLP inference regardless of range. However, `_get_speed_scaling` only produces non-unity output for targets within `PROXIMITY_THRESHOLD = 1.8 m`. Tracks at 2.5–4.0 m consume full inference compute, feature assembly, and normalization with zero contribution to the safety scaling output. Add an early-continue guard (`if track['centroid'][2] > PROXIMITY_THRESHOLD: continue`) before appending to `features_list`, cutting batch size and inference latency in crowded long-range scenes.

### Idea 205: Cross-Track Error and Commanded Velocity Columns in RunLogger
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `current_path_y` (perpendicular cross-track error), `last_vx_cmd`, `last_vy_cmd`, and `vy_rep` are computed on every control tick in drive states but are absent from the CSV columns written by `RunLogger.log()`. These are the most direct metrics for evaluating A/B path-following quality and APF wall-centering effectiveness. Add them as additional fields to `_maybe_log()` in both `DRIVE_TO_B` and `DRIVE_TO_A` states — pure logging additions with zero runtime overhead change to the control path.

### Idea 206: Static Fixture Tagging via EMA Motion Score to Clean Up TTC Loop
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** Tracks representing static furniture (chair legs, desk bases) appear regularly in depth images and accumulate high `visible_count` values, driving them through MLP inference every cycle. Unlike Idea 94 (which masks static objects in image space via LiDAR projection), maintain a lightweight per-track `motion_score = 0.8 * prev_score + 0.2 * (abs(dx) + abs(dy))` updated each frame in `ObstacleTracker.update()`. Tag any track with `motion_score < 0.004 m/frame` sustained for 20+ consecutive frames as `is_static_fixture` and skip it in `_get_speed_scaling` and bypass-initiation checks, without altering the depth extraction pipeline.

### Idea 207: Settle-State Stop Command Deduplication via Published-Flag
**Area:** Script Performance
**Investment:** Extremely Low
**Rationale:** In `SETTLE_1`, `SETTLE_2`, and `SETTLE_3`, `_stop_robot()` is called on every 20 Hz timer tick, publishing 3 zero-velocity Twist messages per call — 60 DDS `/cmd_vel` publishes per second while settled. The Mcnamu_driver_X3 node processes each identically with no effect after the first. Add a `self._motor_is_stopped = False` flag: `_stop_robot()` only publishes when the flag is False and then sets it to True; any non-zero twist publish resets it to False. This reduces settle-state `/cmd_vel` DDS churn from 60 Hz to a single burst per settle entry.

### Idea 208: Adaptive ICP Iteration Count Based on Commanded Speed
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** `_align_scans_icp` runs a fixed `range(3)` iterations regardless of robot velocity. At low commanded speeds (< 0.05 m/s — near-waypoint deceleration or settle-state entry), the scan displacement between consecutive calls is negligible and a single iteration fully converges. At high speeds the full 3 iterations are warranted. Replace the hard-coded `range(3)` with `n_iter = 1 if abs(self.last_vx_cmd) < 0.05 else 3` to reduce ICP CPU cost by two-thirds during slow and stopped segments while preserving full alignment quality at high traverse speeds.

---

## [2026-06-08] Automated Analysis Batch — Ideas 209 to 220

### Idea 209: Cache `start_yaw` Trigonometry Constants at Segment Transitions
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In both `DRIVE_TO_B` (~line 849) and `DRIVE_TO_A` (~line 1046), path distance and cross-track error are computed as `dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)` and `path_y = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)` on every 20 Hz control tick, where `start_yaw` is a constant for the entire duration of each segment (assigned once in the SETTLE state transitions). Pre-caching `self._cos_start_yaw = math.cos(self.start_yaw)` and `self._sin_start_yaw = math.sin(self.start_yaw)` at the assignment point eliminates 4 transcendental function calls per tick — 80 FPU operations per second — replacing them with free float variable lookups.

### Idea 210: Depth Colorization Lazy Gating by Client Subscription State
**Area:** Script Optimization
**Investment:** Low
**Rationale:** `ROS2Bridge._depth_cb` (~line 308) executes the full colorization pipeline (range clipping, min-max normalization, `cv2.applyColorMap`) on every incoming depth frame at camera rate, even when `depth_enabled = False` and no browser client is viewing depth. On the Jetson Orin, this BGR colorization step consumes roughly 3–5 ms per frame. Splitting the callback into two paths — always store the raw float32 array (needed by `VelocityEstimator`), but only compute and store the colorized BGR frame when `depth_enabled = True` — eliminates the dominant CPU cost of depth callback processing during typical operation when visualization is inactive.

### Idea 211: Single `_estimates` Snapshot Per Control Loop Tick
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** Within each 20 Hz `_control_loop` tick during drive states, `_get_speed_scaling()` and `_update_bypass_offset()` each independently acquire `self._estimates_lock` and copy `self._latest_estimates` via `with self._estimates_lock: estimates = list(...)`. Since both are called sequentially within the same tick, this results in two lock acquisitions and two `list()` copies per tick — 40 lock operations and 40 list copies per second. Pre-fetching the snapshot once at the top of the drive-state block in `_control_loop` and passing it directly to both functions eliminates one redundant lock acquisition and one list copy per drive iteration.

### Idea 212: Store `prev_scan_points` Directly as Pre-Converted NumPy Array
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** In `_scan_cb`, current scan points are built as a Python list of `(float, float)` tuples via `.append()` and then stored as `self.prev_scan_points = curr_pts`. In `_align_scans_icp`, the first operation is `P = np.array(prev_pts, dtype=np.float32)` — converting this list to a NumPy array on every ICP call at 20 Hz. Changing the storage to `self.prev_scan_points = np.array(curr_pts, dtype=np.float32) if curr_pts else np.empty((0, 2), dtype=np.float32)` eliminates the per-call `np.array()` allocation and copy from within the ICP function's hot path, requiring only a one-line change in `_scan_cb`.

### Idea 213: Vectorized Continuous Wall Edge Detection via `np.diff` on Front Sector
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** The `_front_is_continuous_wall` flag in `_update_bypass_offset` is determined by iterating all front-sector beams in a Python loop and checking `abs(r - prev_r) > 0.4` per adjacent pair to accumulate `front_has_edges`. Once ranges are converted to a NumPy array (per Idea 188), this can be replaced by extracting the front-sector slice and computing `diffs = np.abs(np.diff(front_ranges)); front_has_edges = bool(np.any(diffs > 0.4))` — a single vectorized call that replaces the Python per-beam loop with C-level SIMD, keeping the wall detection logic identical while executing in microseconds instead of hundreds of Python interpreter steps.

### Idea 214: APF Repulsion Saturation to Prevent Anti-Bypass Interference
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset`, `vy_rep` from the potential field is added unconditionally to `vy_target`. When the robot has set a bypass offset (e.g., `target_lateral_offset = 0.4 m` for a pedestrian), the wall on the bypass side will generate a repulsion force opposing the bypass command, potentially preventing the robot from completing the lateral shift. Adding a direction gate — zeroing `vy_rep` when its sign opposes the active `target_lateral_offset` and its magnitude is less than the bypass offset command — prevents wall repulsion from fighting active bypass maneuvers while still protecting against wall overshoots after the bypass target is reached.

### Idea 215: Bypass Clearance Confirmation Countdown to Suppress Re-Engagement Oscillation
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** When an obstacle clears (both LiDAR and camera confirm the forward path is free), `_update_bypass_offset` immediately resets `target_lateral_offset = 0.0` and `is_paused = False` in a single 20 Hz tick. For pedestrians near the edge of the detection zone or crossing slowly, this causes rapid toggling between bypass and straight modes at 20 Hz, generating oscillatory lateral commands and wheel slip. Adding a `_clear_confirm_count` counter that requires the path to be consistently confirmed clear for 3–5 consecutive ticks (150–250 ms) before resetting to center eliminates oscillatory bypass re-engagement cycles.

### Idea 216: Dual Depth Frame Acquisition via Single `ROS2Bridge` Lock
**Area:** Script Optimization
**Investment:** Low
**Rationale:** In `velocity_estimator._inference_loop` (~line 407), `get_depth_frame()` and `get_raw_depth_frame()` are called sequentially, each independently acquiring `self._lock` in `ROS2Bridge`. Since both are always consumed together in the same inference step, this results in two separate lock-acquire-release cycles per inference iteration at 10 Hz. Adding a `get_depth_frames()` method to `ROS2Bridge` that returns `(coloured_depth, raw_depth)` in a single `with self._lock:` block halves the lock overhead for depth frame retrieval in the estimation pipeline with minimal code change.

### Idea 217: Scale `vy_rep` Proportionally to `max_allowed_offset` in Tight Corridors
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset`, `vy_rep` is independently clamped to `[-0.12, 0.12]` m/s regardless of corridor width, while `max_allowed_offset` (the dynamic bypass ceiling derived from `corridor_width - robot_width`) is applied only to the bypass offset target. In a very tight corridor where `max_allowed_offset ≈ 0`, a full 0.12 m/s `vy_rep` push would drive the robot past the computed corridor bounds. Clamping `vy_rep` to `[-max_allowed_offset * KP_LATERAL, max_allowed_offset * KP_LATERAL]` ensures APF repulsion stays proportionally bounded to available corridor width, preventing overcorrection and oscillation in tight spaces.

### Idea 218: Z-Score Outlier Rejection for Centroid Depth Median Accuracy
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** In `_extract_depth_centroids`, the centroid depth Z is computed as `np.median(valid_depths)` after decimation. In corridor environments, a large contour can include wall pixels that leaked through the morphological filter, biasing the median toward a depth range between the pedestrian and the wall. After the initial median estimate, removing samples where `|depth - median| > 1.5 * std(valid_depths)` before recomputing a clean median produces a tighter Z estimate less contaminated by wall pixels, directly improving the physical accuracy of the 3D centroid coordinates and the displacement features fed to the MLP history window.

### Idea 219: ICP Warm-Start from Previous Residual Correction for Lateral Slip Compensation
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** The ICP in `_scan_cb` is always initialized from the raw odometry delta `(dx_odom_local, dy_odom_local, dtheta)`. For mecanum lateral slip, odometry systematically underestimates actual lateral displacement, and the previous scan's ICP residual `(dx_corr, dy_corr)` captures this persistent bias. Storing the last accepted ICP residual and blending it as a warm-start prior — `initial_dx += alpha * prev_dx_residual` with `alpha ≈ 0.3` — biases the initial alignment toward the persistent slip direction, reducing iterations needed for convergence and improving correction accuracy across consecutive scans during sustained lateral bypass drives.

### Idea 220: Integral Cross-Track Error Term for Steady-State Lateral Drift Elimination
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** The lateral controller `vy_target = (self.target_lateral_offset - path_y) * KP_LATERAL + self.vy_rep` is purely proportional. Mecanum wheel slip and floor friction create a constant lateral disturbance force that the P-only controller balances with a non-zero steady-state `path_y` — meaning the robot consistently drifts off the centerline over long runs. Adding a small integral term `vy_i += KI_LATERAL * (target_lateral_offset - path_y) * dt` (e.g., `KI_LATERAL = 0.1`) with an anti-windup clamp at `±0.05 m/s` would drive steady-state cross-track error to zero, keeping the robot precisely centered on the configured offset across repeated drive segments.

---

## [2026-06-08] Automated Analysis Batch — Ideas 221 to 230

### Idea 221: ICP Accumulated Rotation Delta Bound Clamping
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_align_scans_icp` accumulates `dtheta_corr` across up to 3 iterations and applies the total rotation to `corrected_yaw` without any magnitude guard. In open doorways or sparse scans where point correspondence is poor, spurious rotation corrections of 0.2–0.5 rad can be injected in a single callback — far larger than any physical yaw slip event. Idea 194 already clamps translation deltas; extending it with `if abs(dtheta_corr) > 0.15: dtheta_corr = 0.0` prevents outlier rotations from corrupting `corrected_yaw` while leaving the translation correction intact, adding a single comparison to the post-ICP path in `_scan_cb`.

### Idea 222: Vectorized XY Scan-Point Array Construction from Cached Angle Array
**Area:** Script Optimization
**Investment:** Low
**Rationale:** In `_scan_cb`, valid scan points are built with a Python `for` loop appending `(r * cos(angle), r * sin(angle))` tuples to `curr_pts`. Once Idea 188 (NumPy LaserScan conversion) and Idea 187 (precomputed angle cache) are in place, this loop can be replaced by: `valid = (ranges > 0.15) & (ranges < 4.0) & ~np.isnan(ranges); r_v = ranges[valid]; curr_pts_np = np.column_stack([r_v * np.cos(cached_angles[valid]), r_v * np.sin(cached_angles[valid])])`, building the ICP input array in a single vectorized pass with no Python loop or list appends. Combined with Idea 212's pre-conversion of `prev_scan_points`, this eliminates all Python-level point-cloud construction overhead in the scan callback hot path.

### Idea 223: Hard Physical Clamp on Inverse-Scaled MLP Output Velocities
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** After inverse-scaling predictions (`pred_ms = pred_scaled * scaler_y_scale + scaler_y_mean`) in `_inference_loop`, no upper bound is applied to the resulting `vx`/`vy` values. Severe out-of-distribution inputs — e.g., a large depth contour misregistration or a missing ego-motion compensation frame — can produce predictions of ±5+ m/s, triggering immediate TTC scaling to zero and locking the robot in a prolonged stop. Idea 152 already clamps the *change* in speed between consecutive frames; adding `pred_ms = np.clip(pred_ms, -2.5, 2.5)` immediately after the inverse transform enforces the hard physical plausibility limit (fastest human sprint ≈ 2.5 m/s) as an unconditional output gate with a single one-line change.

### Idea 224: Forward Speed Ramp-Up Delay After Pause Release
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** When `is_paused` transitions from True to False at the end of a bypass maneuver, the forward speed command immediately jumps to its full proportional value on the next 20 Hz tick, creating a speed discontinuity that can cause wheel slip and a brief heading jerk. Add a `_pause_exit_time = None` timestamp; when `is_paused` becomes False, record `time.monotonic()` and cap the forward speed at `base_speed * min(1.0, (now - _pause_exit_time) / 0.3)` for the following 300 ms, providing a smooth velocity ramp-up rather than an instantaneous step change at the moment of obstacle clearance.

### Idea 225: EKF Pose Staleness Guard in `get_robot_pose_and_twist`
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** The `get_robot_pose_and_twist` callback in `server_x3.py` returns the latest `ROS2Bridge._pose_m` and `_twist` with no timestamp check. If `/odom` updates stop arriving (e.g., during `base_node_X3` bringup delay or hardware restart), the frozen stale pose is returned indefinitely and all centroid-to-global transforms use an incorrect robot location, creating phantom velocities on static objects. In `ROS2Bridge._odom_cb`, store `self._odom_stamp = time.monotonic()` on each update; in `get_robot_pose_and_twist`, return `None` if `time.monotonic() - ros_bridge._odom_stamp > 0.5`, causing `_inference_loop` to fall back to `rx_rob = ry_rob = rtheta_rob = 0.0` rather than using a dangerously stale transform.

### Idea 226: ICP Skip During High Lateral Command Velocity
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** The ICP scan-matcher in `_scan_cb` relies on high point-correspondence quality between consecutive scans. During aggressive lateral bypass strafing (`abs(last_vy_cmd) > 0.10 m/s`), side-wall and furniture points shift in perspective rapidly and the ICP convergence basin degrades — yet the 3-iteration loop still runs and integrates noisy corrections into `corrected_x/y/yaw`. Adding a guard at the top of the ICP block in `_scan_cb` (`if hasattr(self, 'last_vy_cmd') and abs(self.last_vy_cmd) > 0.10: use odom fallback`) bypasses the ICP and falls back to the raw odometry delta path (already implemented for sparse scans), preventing lateral-slip noise from corrupting the corrected pose during the maneuvers where odometry-only propagation is actually more reliable.

### Idea 227: Remove Redundant Local `history` Deque from `ObstacleTracker`
**Area:** Script Optimization
**Investment:** Low
**Rationale:** Each track in `ObstacleTracker` maintains two parallel `deque(maxlen=WINDOW_SIZE)` objects: `history` (local camera-frame coordinates) and `history_global` (global map-frame coordinates). In `_inference_loop`, `hist_local` is reconstructed entirely from `history_global` by applying the inverse robot rotation (lines 481–491); the `history` (local) deque is appended at tracking time but never read back during inference. Removing the `history` deque and its `.append()` call in `ObstacleTracker.update()` eliminates one deque allocation per new track, one append per 10 Hz update step per track (up to 5 tracks = 50 appends/second saved), and halves the per-track coordinate storage footprint with zero behavioral change.

### Idea 228: Minimum-Range Safety Halt During ROTATE States
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `ROTATE_180` and `ROTATE_HOME` publish angular velocity commands with no proximity check — the robot will spin into a suddenly-appeared obstacle (e.g., a pedestrian stepping behind the robot during the pause) without any detection. Idea 40 proposes a full swept-volume VO solution; a simpler and immediately deployable guard is: at the top of both rotate-state branches, compute `valid_r = [r for r in self.last_scan_ranges if 0.15 < r < 4.0]` and if `min(valid_r) < 0.22`, publish a zero Twist and return early. This adds a single min-range comparison per 20 Hz tick to both rotation states, preventing in-place collision during turnarounds at negligible CPU cost.

### Idea 229: ICP-Corrected Pose Columns in `RunLogger`
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `RunLogger.log()` and `_maybe_log()` record `robot_x/y/th` from raw EKF odometry (`self.current_x/y/yaw`), but the ICP-corrected pose (`self.corrected_x`, `self.corrected_y`, `self.corrected_yaw`) — which is the actual reference used by all distance and heading controllers — is absent from the CSV. Adding `corrected_x`, `corrected_y`, and `corrected_yaw_deg` as three additional fields to `RunLogger.log()` and `_maybe_log()` enables direct post-run quantification of the ICP correction magnitude (raw odom vs. corrected divergence) per mode, providing a concrete navigation accuracy metric to compare reactive and predictive runs without any runtime overhead change.

### Idea 230: Depth Frame Staleness Gate in `_inference_loop`
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** `_inference_loop` calls `camera.get_raw_depth_frame()` and `camera.get_depth_frame()` every 100 ms without checking when the frame was last written. In the ROS2 mode, `ROS2Bridge._depth_cb` is the writer; if the depth topic stops publishing (camera disconnect, orbbec_depth service restart), the shared memory buffer retains the last valid frame indefinitely. Processing a stale frame creates phantom centroid detections at fixed locations, which the tracker interprets as a stopped pedestrian — generating persistent non-zero MLP output and holding `is_paused = True` in the test script. Track `self._last_depth_write_time` in `_depth_cb` and expose it via `get_depth_frame_age()`; in `_inference_loop`, skip the centroid extraction and set `centroids_m = []` if the frame age exceeds 200 ms, preventing stale data from polluting track history.

---

## [2026-06-08] Automated Analysis Batch — Ideas 231 to 240

### Idea 231: APF `vy_rep` Deadband for Near-Zero Jitter Elimination
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** The potential field repulsion in `_update_bypass_offset` adds `vy_rep` to `vy_target` continuously, including micro-forces of 0.002–0.008 m/s near the `d_safe = 0.55 m` activation boundary. These sub-threshold forces cause the lateral P-controller to oscillate at 20 Hz around the target offset, producing high-frequency noise in `path_y` CSV data that obscures true cross-track error trends. Adding `if abs(self.vy_rep) < 0.012: self.vy_rep = 0.0` before the `vy_target` computation eliminates micro-jitter with a single comparison and zero impact on macro avoidance behavior where repulsion forces exceed the deadband.

### Idea 232: Segment-Specific Rotation Timeout
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `SEGMENT_TIMEOUT = 20.0 s` governs all active-motion states including `ROTATE_180` and `ROTATE_HOME`. A 180° rotation at `MAX_ANGULAR_SPEED = 0.30 rad/s` should complete in ~10 s worst-case; if heading oscillates near the ±180° boundary (a known chattering mode), the robot spins unproductively for the remaining 10+ s before the timeout fires. Introducing a separate `ROTATION_TIMEOUT = 10.0 s` constant and applying it specifically to `ROTATE_180` and `ROTATE_HOME` in the state timeout guard at line 829 catches rotation stalls twice as fast at zero additional CPU cost.

### Idea 233: Settle-State ICP Reference Scan Capture
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** In `_scan_cb`, the ICP reference `prev_scan_points` at each DRIVE segment start is the last scan captured during the prior motion phase, which may contain motion blur and dynamic obstacle points. During SETTLE_1/2/3 the robot is fully stopped and LiDAR produces its cleanest data. Adding a `self._capture_ref_scan_on_next_tick = True` flag set in each SETTLE→DRIVE transition; in `_scan_cb`, when this flag is set, copy `curr_pts` into `prev_scan_points` before running ICP and clear the flag. Each new drive segment then starts from a noise-free stationary reference, dramatically improving first-tick ICP alignment quality.

### Idea 234: Far-Range Voxel Grid Creation at Working Resolution
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_extract_depth_centroids` (~line 218), `far_grid = np.zeros((h_orig, w_orig), dtype=bool); far_grid[::3, ::3] = True` allocates a full `480×640` boolean array and multiplies `far_mask = ... & far_grid` at full resolution, only for both to be immediately discarded via `[::2, ::2]` downsampling. Creating the far-range stride selection directly on the already-downsampled `raw_depth_frame` (e.g., using `[::2, ::2]` with a combined stride) removes one full-resolution bool array allocation and two full-resolution mask multiply operations per 10 Hz inference cycle with identical functional output.

### Idea 235: `_inference_loop` Minimum Sleep Guard for CPU Overrun Prevention
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_inference_loop` (~line 607), `time.sleep(max(0.0, dt - elapsed))` passes zero when inference exceeds `dt = 0.1 s`, causing the thread to spin without any OS yield during brief thermal-throttle overruns on the Jetson. A spinning inference thread starves the ROS2 spin thread and the 20 Hz control loop of CPU time at exactly the worst moment. Changing to `time.sleep(max(0.001, dt - elapsed))` guarantees a minimum 1 ms scheduler yield per cycle — a single character change that prevents runaway CPU consumption during overruns.

### Idea 236: `visible_count`-Scaled Confidence Weight on MLP Output Speed
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** Tracks that just cleared the 3-frame initiation gate (Idea 109) have `WINDOW_SIZE - 3 = 7` history frames padded with zeros, meaning the MLP receives heavily distorted input that systematically overestimates pedestrian speed. Multiplying the predicted `speed` (and proportionally `vx`/`vy`) by `min(1.0, visible_count / WINDOW_SIZE)` in `_inference_loop` ramps each track's TTC contribution from 30% at gate entry to 100% at full history maturity, suppressing false TTC braking from fresh detections without any model retraining.

### Idea 237: Scan Minimum-Points Gate Increase for ICP Reliability
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_scan_cb` invokes ICP whenever `len(curr_pts) >= 10 and len(self.prev_scan_points) >= 10`. A 10×10 correspondence matrix has ~100 elements — statistically insufficient for reliable 2D rigid-body estimation in corridor geometry. In practice, the Yahboom X3 always operates in enclosed spaces with 80–200 valid LiDAR returns. Raising the threshold to `>= 40` valid points (falling through to the existing odometry-only path otherwise) filters the rare degenerate open-space or scan-initialization scans that currently inject noisy corrections into `corrected_x/y/yaw`.

### Idea 238: Per-Obstacle Speed EMA Smoothing for TTC Stability
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** MLP output `speed` for each track is used directly in `_get_speed_scaling` without inter-frame smoothing. Frame-to-frame speed can jitter ±0.15 m/s due to depth contour fluctuations, causing TTC speed scaling to fluctuate even though Idea 122's aggregate EMA provides only partial suppression. Maintaining a per-track `speed_ema` dict in `_inference_loop`, updated as `speed_ema[tid] = 0.6 * new_speed + 0.4 * prev_ema`, and using it as the speed source for TTC calculations provides per-obstacle noise rejection with ~100 ms time constant — complementary to but distinct from Idea 122's aggregate output EMA.

### Idea 239: Blocked-Time Counter Explicit Reset on Drive-Leg Transition
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `self.blocked_time` is incremented in both `DRIVE_TO_B` and `DRIVE_TO_A` but is never explicitly zeroed at the SETTLE_2→DRIVE_TO_A transition. If a late-cycle blocked-time increment occurs near the end of `DRIVE_TO_B` (e.g., when paused near WP-B), the counter carries a non-zero value into `DRIVE_TO_A`, potentially reaching the 8-second recovery threshold within seconds of starting the return leg without any actual blockage. Adding `self.blocked_time = 0.0` in the `SETTLE_2` block alongside the existing `self.is_paused = False` reset (~line 1034) guarantees a clean counter for each new drive leg.

### Idea 240: Lateral P-Gain Scheduling Based on Proximity to Target Offset
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** `KP_LATERAL = 0.8` applies uniformly across the full lateral error range. At large errors (|error| > 0.3 m, during initial bypass engagement), this gain drives the robot toward the target rapidly but with overshoot past the offset that can trigger the opposite wall's repulsion. At small errors (|error| < 0.05 m, fine centering), a lower gain is more appropriate. Implementing a simple two-regime gain schedule — `kp_eff = 0.8 if abs(lateral_err) > 0.10 else 0.4` — in the `vy_target` computation in both `DRIVE_TO_B` and `DRIVE_TO_A` improves bypass settling quality and reduces overshoot oscillation without requiring derivative control.

---

## [2026-06-09] Automated Analysis Batch — Ideas 253 to 262

### Idea 253: Forward-Scale Bypass Denominator Tied to Active `BYPASS_OFFSET`
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `DRIVE_TO_B` and `DRIVE_TO_A` (~lines 916–917 and 1108–1109 of `ab_comparison_test.py`), the diagonal bypass forward-speed blending uses `forward_scale = 1.0 - max(0.0, min(0.8, lateral_error / 0.5))` with `0.5` hardcoded. When Idea 110 dynamically reduces `BYPASS_OFFSET` to 0.10 m in tight corridors, `lateral_error` never exceeds 0.10 m, so `lateral_error / 0.5 = 0.20` maximum and `forward_scale` never falls below 0.84 — the diagonal speed reduction barely activates. Replacing the hardcoded `0.5` with `max(0.05, BYPASS_OFFSET)` makes the forward-speed blending proportional to the actual bypass target distance, ensuring smooth diagonal profiling whether the active offset is 0.10 m or 0.40 m.

### Idea 254: Redundant `_get_speed_scaling()` Call Elimination in Diagnostic Block
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** Inside `_update_bypass_offset` in `ab_comparison_test.py`, the 2 Hz debug print block at ~line 494 calls `self._get_speed_scaling()` to display the live speed scale. The same method is also called in the main drive control code at lines 906 and 1098, meaning on every debug-print tick the scaling computation runs twice — two `self._estimates_lock` acquisitions, two `list(self._latest_estimates)` copies, and two full pedestrian-loop iterations. Caching the speed_scale result from the control code path and passing it as a parameter to the diagnostic print eliminates one full redundant execution of the most frequently-used estimator path per debug tick.

### Idea 255: Scan-Dirty Flag to Skip Bypass Sector Computation Between LiDAR Updates
**Area:** Script Optimization
**Investment:** Low
**Rationale:** `_update_bypass_offset()` is called at 20 Hz by `_control_loop`, but `self.last_scan_ranges` only updates at ~8 Hz via `_scan_cb`. The expensive Python beam-loop (sector clearances, edge detection, `min_forward_lidar`, APF forces) re-processes identical scan data on ~12 of every 20 control ticks. Setting a `self._scan_dirty = True` flag in `_scan_cb` and clearing it at the start of `_update_bypass_offset` allows the function to return cached clearance, blockage, and `vy_rep` values on ticks where no new scan arrived, cutting scan-loop CPU by ~60% while still re-evaluating camera estimates every tick.

### Idea 256: `blocked_time` Column in RunLogger for Recovery Event Traceability
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `RunLogger._maybe_log()` records `segment`, `n_obstacles`, and `min_obstacle_dist`, but not `self.blocked_time` — the cumulative blocked-duration counter. Post-run CSV analysis cannot distinguish a brief pedestrian pause from a full 8-second recovery event without reconstructing `blocked_time > 8.0` transitions manually across rows. Adding `blocked_time` as a column to `_maybe_log()` captures the full recovery pressure timeline, enabling direct per-mode measurement of how many backing maneuvers occurred and how long the robot spent approaching the 8-second recovery threshold.

### Idea 257: ICP Skip Gate Based on Odom Delta Magnitude for Stationary Phases
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** In `_scan_cb` of `ab_comparison_test.py`, the odom delta `(dx_odom_local, dy_odom_local, dtheta)` is computed before calling `_align_scans_icp`. When the total magnitude `abs(dx_odom_local) + abs(dy_odom_local) + abs(dtheta) < 0.003`, the robot hasn't moved meaningfully and any ICP correction is noise-dominated. Adding an early-return that propagates the odom delta directly to `corrected_x/y/yaw` for near-zero motion frames saves 3× O(N²) distance matrix computations during settle states and blocked pauses — which can constitute 30–50% of total run time — and is distinct from Idea 208 (commanded-speed-based iteration count) and Idea 189 (convergence-criterion early exit).

### Idea 258: Pre-Computed APF Normalization Constant to Eliminate Per-Tick Divisions
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_update_bypass_offset` (~lines 482 and 487), the APF repulsion formula evaluates `1.0 / ((d_safe - d_min) * (d_safe - d_min))` — numerically `1.0 / (0.33 * 0.33) = 9.183` — on every 20 Hz tick for both left and right walls. Since `d_safe = 0.55` and `d_min = 0.22` are compile-time constants, this term never changes at runtime. Pre-computing `self._apf_baseline = 1.0 / (d_safe - d_min) ** 2` in `__init__` and substituting it into both `f_rep_left` and `f_rep_right` formulas replaces 2 floating-point divisions per tick with a float variable read, with zero behavioral change.

### Idea 259: `detections_fn()` Hoisted Outside Per-Contour Loop for Gating Efficiency
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_extract_depth_centroids`'s raw-depth path (~line 271 of `velocity_estimator.py`), `self.detections_fn()` is called inside the `for cnt in contours:` loop — once per detected contour per inference cycle. With up to 5 contours, this causes 5 closure invocations and 5 `person_boxes` list comprehensions per 10 Hz cycle even though detections do not change between contours in the same frame. Hoisting `detections = self.detections_fn()` and `person_boxes = [...]` extraction to just before the loop eliminates up to 4 redundant fetches per inference cycle, cutting Visual-LiDAR gating overhead by up to 80%.

### Idea 260: Quadratic Deceleration Profile in Final 0.30 m Approach Zone
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** In both drive states, forward speed is `max(MIN_LINEAR_SPEED, min(max_speed, dist_error * KP_DIST))`. This proportional law maintains nearly constant speed down to `dist_error ≈ 0.10 m` where the output reaches `MIN_LINEAR_SPEED = 0.06 m/s` with a sharp knee — the robot arrives at the waypoint traveling near minimum speed with no anticipatory smoothing. Replacing the proportional term with `dist_error * KP_DIST * min(1.0, dist_error / 0.30)` introduces a smooth quadratic decay in the final 0.30 m (e.g., 0.67× at 0.20 m error, 0.33× at 0.10 m), reducing waypoint overshoot and final position error in both reactive and predictive modes.

### Idea 261: Time-Based Track Age Expiry for Throttle-Robust Track Persistence
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** `ObstacleTracker` expires tracks when `age > max_age = 10 frames` (1.0 s at 10 Hz). Under Jetson thermal throttling, `_inference_loop` may run at 5–6 Hz, making 10 frames equal 1.7–2.0 s — extending lifetimes unpredictably and allowing stale tracks to contribute to TTC calculations. Replacing the frame-count `age` increment with `time.monotonic()` timestamps (storing `last_seen` per track and expiring when `now - last_seen > max_age_seconds`) makes track persistence invariant to inference scheduling jitter, distinct from Idea 248 which varies `max_age` between modes.

### Idea 262: RunLogger Incremental File Write for Repeat-Mode Memory Reduction
**Area:** Script Optimization
**Investment:** Low
**Rationale:** In `repeat=True` mode, `RunLogger.log()` appends row dicts to `self.rows` in memory for the full leg duration, with `save()` called only at `ROTATE_HOME` completion. A 2-minute forward+return leg at 10 Hz accumulates ~1200 dict objects in memory, all written in a single burst. Opening the CSV file in `RunLogger.__init__`, writing the header immediately, and calling `writer.writerow(row)` in `log()` on each append eliminates peak memory, distributes I/O evenly across the leg, and prevents complete data loss if the process is killed or the robot is e-stopped mid-run.

---

## [2026-06-09] Automated Analysis Batch — Ideas 263 to 272

### Idea 263: `normalize_angle` O(1) Modulo Formula Replacement
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `ab_comparison_test.py` lines 64–69, `normalize_angle` uses two `while` loops to wrap an angle into `[-π, π]`. At 20 Hz, this function is called at least 6–8 times per control tick (odom callback, heading error, scan angle offset in `_update_bypass_offset`, ICP accumulation), totalling 120–160 calls per second. Replacing the while-loops with the single-expression formula `return ((angle + math.pi) % (2 * math.pi)) - math.pi` handles all inputs in O(1) using a single modulo and subtraction, eliminating worst-case multi-iteration loop overhead at zero precision loss.

### Idea 264: `_min_forward_lidar` Column in RunLogger for Wall-Proximity Analysis
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `self._min_forward_lidar` is updated every control tick in `_update_bypass_offset` as the closest LiDAR return in the ±20° forward cone, and is already used for the static-wall early-stop guard. It is never written to the CSV. Adding `min_forward_lidar: round(self._min_forward_lidar, 3)` to `RunLogger.log()` and the `_maybe_log()` call sites provides a per-row record of how close the robot came to walls while driving, enabling direct post-run A/B comparison of whether predictive mode maintains greater forward clearance than reactive mode.

### Idea 265: Per-Segment RMS Cross-Track Error Summary Row in RunLogger
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** Path-following accuracy is the primary navigation metric for the A/B comparison, yet no single number currently characterises each leg's straightness — deriving it requires averaging all `current_path_y` values across hundreds of CSV rows in post-processing (only possible after Idea 205 adds that column). Maintaining running accumulators `_path_y_sum_sq` and `_path_y_count` throughout each drive state and writing a single `"SEGMENT_SUMMARY"` row with `path_y_rms = sqrt(_path_y_sum_sq / _path_y_count)` at the drive-state exit transition provides a ready-made per-mode, per-leg path-straightness scalar with no additional runtime overhead.

### Idea 266: Adaptive APF Repulsion `eta` Scaled by Current Forward Speed
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset` the APF constant `eta = 0.03` is applied uniformly regardless of robot speed. At high forward speed (`last_vx_cmd ≈ MAX_LINEAR_SPEED`), a strong repulsion force produces abrupt lateral velocity step-changes that cause mecanum wheel slip and corrupt the ICP reference pose; at low approach speed near a waypoint, stronger repulsion improves centering accuracy. Replacing the constant with `eta_eff = 0.03 * max(0.5, 1.0 - abs(self.last_vx_cmd) / MAX_LINEAR_SPEED)` scales repulsion from 0.015 at full forward speed to 0.03 at rest, balancing slip prevention at high speed with tight corridor centering during slow approach.

### Idea 267: Waypoint Arrival Hysteresis Band to Prevent ICP-Noise False Arrivals
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** The arrival condition `if dist_error <= DIST_TOLERANCE: stop` in `DRIVE_TO_B` and `DRIVE_TO_A` uses a single-tick check. If ICP scan corrections cause `corrected_x/y` to fluctuate across the `DIST_TOLERANCE = 0.05 m` boundary (e.g., alternating between 0.04 m and 0.06 m), the robot may prematurely enter `SETTLE` states before actually reaching the waypoint, corrupting the run. Adding a `self._arrival_confirm_count` integer that increments when `dist_error <= DIST_TOLERANCE` and resets otherwise, transitioning to settle only when `_arrival_confirm_count >= 2`, requires the condition to hold for two consecutive 20 Hz ticks (100 ms) before committing — filtering ICP micro-fluctuations while adding negligible delay on genuine arrivals.

### Idea 268: `n_icp_inliers` Column in RunLogger for Scan Match Quality Monitoring
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_align_scans_icp` computes `np.sum(valid)` (inlier count) on the final ICP iteration but discards this value. Storing it as `self._last_icp_inliers = int(np.sum(valid))` at the end of `_scan_cb` and adding it as a column to `RunLogger.log()` would reveal scan match degradation events (low inlier counts from sparse doorway scans or featureless walls) in the per-run CSV, allowing post-run A/B correlation between ICP quality and navigation drift magnitude — a diagnostic that is currently impossible without re-running ICP offline.

### Idea 269: `front_has_edges` Minimum Edge-Count Gate for Specular-Robust Wall Classification
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `_update_bypass_offset`, `front_has_edges` is set to `True` if *any single beam* in the front sector shows a range discontinuity > 0.4 m versus its neighbour. A single specular return from a shiny floor tile or metal frame can set this flag on an otherwise flat wall, preventing `is_continuous_wall = True` and re-enabling camera-based bypass for a genuine static boundary. Replacing the single-hit boolean with a counter `front_edge_count` and requiring `front_edge_count >= 2` before setting `front_has_edges = True` makes the classification robust to isolated specular returns; a genuine pedestrian silhouette produces 4–6 edge transitions at both body edges while a lone reflective spike contributes only one.

### Idea 270: Speed-Scale EMA Minimum Per-Tick Recovery Increment
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `_get_speed_scaling` (~line 784), when `speed_scale > smooth_speed_scale` (obstacle just cleared), recovery follows `smooth_speed_scale = 0.15 * speed_scale + 0.85 * smooth_speed_scale`. Starting from `smooth_speed_scale = 0.0` with `speed_scale = 1.0`, this EMA reaches only 0.075 after tick 1, 0.139 after tick 2, … requiring ~14 ticks (700 ms at 20 Hz) to reach 0.90. After obstacle clearance the forward path is open but the robot crawls for nearly a second. Adding `self.smooth_speed_scale = max(recovered, self.smooth_speed_scale + 0.015)` guarantees at least 1.5% per-tick forward progress on recovery — reaching full speed in ≤ 5 ticks (250 ms) while leaving the instant-brake behaviour unmodified, and is orthogonal to Idea 224 which applies a separate time-gated cap at pause exit.

### Idea 271: State Transition Marker Rows in RunLogger for Unambiguous Segment Boundaries
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `_maybe_log()` is called inside active drive states but never at state transitions. Post-run CSV analysis must infer segment boundaries by finding rows where `segment` changes from one value to the next, which fails if the final row of a drive state was missed (throttled log or early exit). Adding a single call to a lightweight `_log_event(label)` method that writes one CSV row with `segment = label` (e.g., `"EVENT_SETTLE1_START"`) and all numeric fields forwarded from the most recent values at each `DRIVE_TO_B → SETTLE_1`, `SETTLE_1 → ROTATE_180`, etc. transition provides unambiguous delimiters that survive any log-throttling gap.

### Idea 272: `_stop_robot` Reduced from 3-Publish Loop to Single Publish
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `_stop_robot()` in `ab_comparison_test.py` (lines 654–657) publishes three identical zero-velocity `Twist` messages in a tight `for _ in range(3):` loop. With ROS2 reliable QoS (`depth=10`), a single publish is guaranteed to be delivered to the `/cmd_vel` subscriber via DDS, making the 3-publish pattern a carryover from ROS1 best-effort UDP practices. Called at 20 Hz from SETTLE states (before Idea 207 is applied), this generates 60 redundant DDS publish operations per second. Reducing to a single publish inside `_stop_robot` cuts unnecessary middleware overhead and is orthogonal to Idea 207, which prevents repeated *calls* to `_stop_robot` in settle states.

---

## [2026-06-09] Automated Analysis Batch — Ideas 273 to 282

### Idea 273: ICP Disabled During Rotation and Settle States
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_scan_cb` runs the full 3-iteration ICP unconditionally at ~8 Hz, including during `ROTATE_180`, `ROTATE_HOME`, and all three `SETTLE` states. During rotation each successive scan is offset by ≥15° from the previous; the ICP initial guess (`initial_dtheta` from raw odom) is 0.1–0.3 rad, making nearest-point correspondence poorly aligned and producing large spurious dx/dy corrections that are integrated into `corrected_x/y`. Adding `if self.state not in (self.DRIVE_TO_B, self.DRIVE_TO_A): apply raw odom delta only` in `_scan_cb` limits ICP to active drive phases where lateral mecanum slip is meaningful, eliminating cross-contamination of the drift reference pose used throughout each subsequent drive segment.

### Idea 274: `corrected_yaw` Soft Re-Sync to EKF Heading at Drive-Segment Entry
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** ICP noise accumulated during rotation and settle states can bias `corrected_yaw` away from the true robot heading before a drive segment starts. Because heading error `normalize_angle(target_yaw - corrected_yaw)` is the direct input to the yaw PID correction, a 0.05 rad ICP-induced bias injects a persistent yaw correction torque throughout the entire drive segment, causing the robot to arc slightly rather than drive straight. Adding `if abs(normalize_angle(self.corrected_yaw - self.current_yaw)) > 0.05: self.corrected_yaw = self.current_yaw` at each SETTLE→DRIVE transition (lines ~815 and ~1031 in `ab_comparison_test.py`) resets rotation-phase ICP yaw noise to the EKF ground truth without discarding the lateral (dx/dy) correction that ICP is designed to provide.

### Idea 275: Speed-Proportional `WALL_STOP_DIST` Lookahead Buffer
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `WALL_STOP_DIST = 0.20 m` is applied identically at all forward speeds. The practical stopping margin must cover the 125 ms LiDAR scan period plus ~50 ms control tick latency; at the current `MAX_LINEAR_SPEED = 0.20 m/s` this equates to ~0.035 m of uncontrolled travel, leaving comfortable margin. If `MAX_LINEAR_SPEED` or `KP_DIST` is increased in future tuning, the fixed threshold becomes insufficient. Replacing with `effective_stop_dist = max(0.20, self.last_vx_cmd * 0.25)` (250 ms speed lookahead) guarantees a proportional stopping buffer that is automatically correct across all speed configurations without changing any existing logic at current speeds.

### Idea 276: Bypass Direction Selection via Pedestrian Lateral Velocity Projection
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset`, the bypass side is chosen using the obstacle's current lateral position (`if ry >= 0.0: prefer right`). For a moving pedestrian, the robot may select the exact side the pedestrian is heading toward, resulting in a converging approach. Replacing the current-position check with `ry_projected = ry + est.get('vy', 0.0) * 0.5` (projecting lateral position 500 ms forward using the MLP-estimated `vy`) before the side selector directs the robot toward the side the pedestrian is vacating, directly leveraging the velocity estimate that distinguishes predictive mode from reactive mode and producing a measurably safer bypass trajectory in the A/B comparison.

### Idea 277: YOLO Person Detection Veto for `is_continuous_wall` Stationary Pedestrian Case
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** `is_continuous_wall` is set True when LiDAR shows blockage without range discontinuities and no `has_dynamic_pedestrian` (speed > 0.15 m/s). A stationary pedestrian standing directly in front presents a near-uniform LiDAR range profile (no edges) and has speed = 0.0, causing them to be misclassified as a static wall, suppressing camera-based bypass and halting the robot indefinitely. Adding a check against `self.detections_fn()` in the `is_continuous_wall` computation — if any YOLO `person` bounding box is present in the forward frustum, force `is_continuous_wall = False` regardless of edge count — correctly identifies stationary people as bypassable obstacles using the existing visual pipeline already available via `detections_fn`.

### Idea 278: LiDAR Range-Rate Forward Sector TTC Supplement for Reactive Mode
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** `_get_speed_scaling` relies entirely on MLP velocity estimates from `_latest_estimates`, which are unavailable in reactive mode and require 3 track frames to initialise. The LiDAR provides an independent radial closure rate: for beams in the ±20° forward cone, `range_rate_i = (prev_range_i - curr_range_i) / dt_scan` gives the approach speed of each point without depth camera or MLP involvement. Storing `self._prev_forward_ranges` in `_scan_cb` and computing `lidar_ttc = min(curr_range_i / max(0.01, range_rate_i))` over closing beams provides a reactive-mode TTC supplement that triggers `smooth_speed_scale` reduction before the camera tracker initialises, closing a safety gap during track warm-up and providing an independent redundancy check in predictive mode.

### Idea 279: `last_vy_cmd` Reset After Recovery Backing Maneuver
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** After the 8-second recovery backing maneuver in `DRIVE_TO_B` and `DRIVE_TO_A` (lines ~944–951 and ~1134–1141), `self.target_lateral_offset` and `self.is_paused` are reset to 0/False, but `self.last_vy_cmd` retains its value from the last bypass attempt (potentially ±0.10–0.15 m/s). On the next control tick the rate limiter (`MAX_LATERAL_ACCEL = 0.5 m/s²`) ramps `vy_cmd` from `last_vy_cmd` toward 0 over ~6 ticks (300 ms), causing the robot to briefly strafe sideways while simultaneously trying to resume forward drive post-recovery. Adding `self.last_vy_cmd = 0.0` immediately after the backing loop completes eliminates this unintended post-recovery lateral command with a single one-line fix.

### Idea 280: `is_dynamic_pedestrian` Hysteresis Latch to Prevent Mid-Bypass Corridor Narrowing
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** `is_dynamic_pedestrian` is recomputed from scratch on each call to `_update_bypass_offset` by checking `speed > 0.15 m/s`. If a pedestrian briefly decelerates below 0.15 m/s mid-bypass (changing direction, pausing), the corridor immediately collapses from dynamic (`BYPASS_OFFSET = 0.4 m`) to static (`BYPASS_OFFSET = 0.25 m`), clamping an active bypass offset from 0.4 m to 0.25 m and potentially driving the robot back toward the obstacle before clearing it. Adding a `self._dynamic_ped_latch_frames` counter that holds `is_dynamic_pedestrian = True` for 10 additional frames (1 second at 10 Hz estimates) after the last `speed > 0.15` detection prevents mid-bypass corridor collapse for pedestrians changing pace, ensuring the wider evasion envelope is maintained until the bypass is complete.

### Idea 281: Approach-Velocity-Scaled `AVOIDANCE_FORWARD` for Earlier Bypass Initiation
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** `AVOIDANCE_FORWARD = 1.8 m` is a fixed camera-based bypass trigger horizon. For a pedestrian approaching at 1.0 m/s with the robot at 0.20 m/s (closure rate 1.2 m/s), at 1.8 m separation only 1.5 s remain to complete a bypass — which may be insufficient for the lateral rate limiter to reach full offset and return to center. Computing `avoidance_dist_dyn = 1.8 + max(0.0, (-est.get('vx', 0.0) - self.last_vx_cmd) * 0.4)` (where negative `est_vx` means the pedestrian is approaching, and 0.4 s is a lookahead factor) extends the trigger distance proportionally to approach speed, allowing the robot to begin strafing earlier for fast pedestrians while leaving the 1.8 m default unchanged for slow or receding targets.

### Idea 282: Wall-Stop and Bypass-Suppression Event Counters in RunLogger
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_front_is_continuous_wall` activations (which suppress camera bypass and trigger early-stop transitions) are never counted or logged. Post-run CSV analysis cannot determine how many times each mode correctly classified a genuine wall versus misclassified a stationary pedestrian or wide obstacle, making it impossible to audit the wall filter's impact on A/B run outcome. Adding `self._wall_stop_count` (incremented at each early-stop branch in `DRIVE_TO_B`/`DRIVE_TO_A`) and `self._bypass_suppressed_count` (incremented when `is_continuous_wall = True` blocks a camera-based bypass that would otherwise fire) and appending them to `RunLogger.save()` as summary fields provides direct mode-by-mode classification frequency data at zero per-tick overhead.

---

## [2026-06-09] Automated Analysis Batch — Ideas 241 to 252

### Idea 241: ICP Computation Offloaded to a Dedicated Background Thread
**Area:** Navigation Accuracy
**Investment:** Medium
**Rationale:** `_scan_cb` in `ab_comparison_test.py` runs the full 3-iteration ICP (including `O(N²)` distance matrix construction) synchronously inside the ROS2 callback thread at ~8 Hz. This blocks the single-threaded ROS2 executor for the duration of each ICP run, potentially delaying `/odom` and `/scan` message processing and causing control loop jitter. Moving the ICP computation to a dedicated background thread — writing the accepted result into `self.corrected_x/y/yaw` under a lock — frees the scan callback to return immediately, eliminating executor starvation and scan queue buildup at the cost of one extra Python lock and thread.

### Idea 242: Forward-Sector LiDAR Beam Count Gate for Wall-vs-Obstacle Classification
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `_front_is_continuous_wall` is set to True whenever `front_blocked_by_lidar AND not front_has_edges AND no_dynamic_pedestrian`. However, a large but finite obstacle (a wide trolley or storage cabinet presenting a uniform-range front face) also satisfies this condition and is misclassified as an infinite wall, suppressing the camera-based bypass. A genuine corridor end-wall produces 30+ adjacent beams at near-identical range; a discrete wide object produces far fewer. Adding a minimum-beam count check — e.g., requiring at least 15 beams in the ±20° forward cone to register within `min_forward_lidar + 0.05 m` before setting `is_continuous_wall = True` — makes wall classification more selective without additional sensors.

### Idea 243: ObstacleTracker History Deque Reset at Repeat-Mode Run Boundaries
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** In `repeat=True` mode, at the `ROTATE_HOME → DRIVE_TO_B` restart in `ab_comparison_test.py`, the `ObstacleTracker` inside `VelocityEstimator` retains `history_global` deques populated during the previous leg. A pedestrian who stood at WP-B during the forward run has a history that described the robot approaching head-on; on the return leg the same global positions now represent the robot receding, yet the MLP feature reconstruction from `history_global` inverts the displacement directions. Adding a `clear_histories()` method to `ObstacleTracker` that empties each track's `history_global` deque (without dropping the track itself) and calling it via the `set_velocity_estimation` toggle at each run restart prevents cross-leg history contamination in the MLP input window.

### Idea 244: Round-Trip Pose Drift Magnitude Logged at Run Completion
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** At the end of each round-trip (when `ROTATE_HOME` reaches `YAW_TOLERANCE`), `corrected_x` and `corrected_y` represent the ICP-corrected final position, while `init_x` and `init_y` represent the true physical start. Their Euclidean distance `drift_m = sqrt((corrected_x - init_x)^2 + (corrected_y - init_y)^2)` is the accumulated loop-closure position error for the entire run. Appending this single scalar as a final row in the CSV (or as a header comment in `RunLogger.save()`) provides a direct, per-run quantitative comparison of mecanum slip + ICP correction quality between reactive and predictive modes — currently impossible to derive without manual coordinate reconstruction.

### Idea 245: EMA Smoothing on `_min_forward_lidar` to Prevent Specular False Early-Stops
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `self._min_forward_lidar` is updated in `_update_bypass_offset` as the raw minimum LiDAR range in the ±20° forward cone. A single specular spike from a shiny floor tile or metallic surface can momentarily read 0.15–0.18 m, satisfying `_min_forward_lidar < WALL_STOP_DIST (0.20 m)` and triggering an early stop even though the robot is 1.5+ m from any real wall. Applying a two-sample EMA `self._min_forward_lidar = 0.65 * raw_min + 0.35 * self._min_forward_lidar` before the early-stop check filters isolated sub-frame spikes while still responding to a genuine approaching wall within 2–3 scan cycles (~375 ms at 8 Hz).

### Idea 246: ICP Pose Corruption Detection via Negative `dist_error` Monitor
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** If `_align_scans_icp` produces a spurious large correction that teleports `corrected_x/y` forward past the waypoint, `dist_error = waypoint_b_dist - dist_travelled` can flip negative at the very start of a drive segment before the robot has moved. Adding a guard `if dist_error < -(DIST_TOLERANCE * 3): [warn + reset corrected_x/y to current_x/y]` in both `DRIVE_TO_B` and `DRIVE_TO_A` catches ICP-induced pose teleportation and falls back to the raw EKF odometry position. Without this guard, the control loop would immediately satisfy the arrival condition and skip an entire drive leg, corrupting the run.

### Idea 247: `is_paused` Column in RunLogger for Drive-vs-Wait Segment Analysis
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `RunLogger.log()` records `segment` (coarse state name), `n_obstacles`, and `min_obstacle_dist`, but not whether the robot was actively driving or blocked by an obstacle at that tick. Post-run CSV analysis cannot distinguish between "robot moving toward WP-B" and "robot sitting stationary behind a pedestrian" without checking `min_obstacle_dist < threshold` as a proxy — which confuses stationary waits in narrow corridors with genuine obstacle pauses. Adding `self.is_paused` as a boolean column to `_maybe_log()` directly labels each row, enabling accurate per-run computation of total blocked time, pause count, and clearance metrics segmented by mode.

### Idea 248: Dynamic `ObstacleTracker.max_age` Scaling Based on Estimation Mode
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** `ObstacleTracker.max_age = 10` frames means tracks are dropped after 1 second of non-detection (at 10 Hz), regardless of whether velocity estimation is active. In reactive mode (`estimation_enabled=False`), tracks are never used for TTC scaling so persistence is irrelevant. In predictive mode, a pedestrian walking through a depth camera dead zone (e.g., passing directly beside the robot) may disappear for 0.8–1.2 s; losing the track resets TTC to 1.0 (no slowdown) precisely when proximity risk is highest. Setting `max_age = 20` when `estimation_enabled=True` and `max_age = 10` otherwise (exposed as a method `set_max_age(n)` called from `_set_estimator_mode`) doubles track persistence only when it affects safety, at zero additional CPU cost.

### Idea 249: Settle-State Physical Stop Confirmation via Corrected-Pose Delta Gate
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** After `_stop_robot()` publishes three zero-Twist commands, the robot coasts due to motor and wheel inertia; `SETTLE_DURATION = 1.0 s` is a fixed blind wait. If the robot arrived at WP-B at close to `MAX_LINEAR_SPEED = 0.20 m/s`, it can coast 10–15 cm during the settle window, causing the start of ROTATE_180 to use a reference heading captured from a still-moving robot. Gating SETTLE_1/2/3 exit on `abs(corrected_x - settle_entry_x) + abs(corrected_y - settle_entry_y) < 0.005 m over the last two ticks` (sampled in the settle branch) plus a 0.5 s minimum ensures mechanical stop before capturing `start_yaw` and `target_yaw` for the next segment.

### Idea 250: Depth Centroid Vertical Position Gate to Reject Floor-Level Ghost Detections
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** In `_extract_depth_centroids` (raw depth path), `y_m = (cy - h / 2.0) * Z / fx` computes the vertical position of the centroid in metres relative to the camera center. Floor reflections, textured floor tiles, and low obstacles (door stoppers, cable runs) produce centroids at large positive `y_m` values (floor is below the camera, so cy > h/2). A simple gate `if y_m > 0.4 * Z: continue` (rejecting centroids that are more than 40% of their depth below camera centre height, approximately below ankle level) eliminates floor-level ghost detections without ground-plane RANSAC, reducing false TTC triggers and unnecessary tracking state from non-pedestrian floor artifacts.

### Idea 251: Cached `cos_r`/`sin_r` with Change-Threshold Refresh in `_inference_loop`
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** At the start of `_inference_loop` every 100 ms, `cos_r = math.cos(rtheta_rob)` and `sin_r = math.sin(rtheta_rob)` are recomputed unconditionally. When the robot is stationary (settle states, blocked pauses), `rtheta_rob` is constant for seconds at a time, repeating identical trig evaluations. Caching `self._cached_cos_r`, `self._cached_sin_r`, and `self._cached_theta` and recomputing only when `abs(rtheta_rob - _cached_theta) > 0.001 rad` converts these from per-cycle trig calls to a single floating-point comparison per cycle during stationary phases, which constitute a significant fraction of the total inference thread runtime during paused/blocked events.

### Idea 252: Binary WebSocket Message Pre-Filter in AB Test Estimator Receive Loop
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_set_estimator_mode`'s async receive loop in `ab_comparison_test.py`, every WebSocket message is passed directly to `json.loads(message)`. The server (`server_x3.py`) broadcasts binary camera frames at up to 20 Hz on the same connection, which are not JSON and trigger `json.JSONDecodeError` or `UnicodeDecodeError` exceptions caught silently by `except Exception: pass`. Python exception handling (stack frame construction, traceback materialization) adds ~10–30 µs overhead per exception — at 20 Hz binary frames this accumulates to 0.2–0.6 ms/s of unnecessary exception-path CPU in the estimator receive thread. Adding `if isinstance(message, bytes): continue` as the first line inside `async for message in ws` eliminates all exception overhead with a single type check.

---

## [2026-06-09] Automated Analysis Batch — Ideas 283 to 292

### Idea 283: Predictive Bypass Pre-Initiation via Forward-Projected Pedestrian Position
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset()`, the `is_dynamic_pedestrian` flag (lines 409–422) widens avoidance corridors but never uses the velocity vector to project where the pedestrian will be in 0.5–1.0 s. By computing `rx_future = rx + vx * 0.5` and `ry_future = ry + vy * 0.5` and checking if the projected position enters the `AVOIDANCE_FORWARD × AVOIDANCE_LATERAL` box, the robot can begin setting `target_lateral_offset` while the person is still outside the current LiDAR trigger range. This gives the lateral rate limiter (MAX_LATERAL_ACCEL = 0.5 m/s²) sufficient time to reach full offset before the pedestrian arrives, reducing close-range emergency stops in predictive mode.

### Idea 284: Range-Adaptive `alpha_z` EMA Filter Based on Centroid Depth Bin
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** `ObstacleTracker` applies a fixed `alpha_z = 0.7` EMA to all centroid depths regardless of range (line 49 of `velocity_estimator.py`). At close range (Z < 1.5 m), Astra Pro depth accuracy is high and a higher alpha (0.85, less smoothing) preserves responsiveness; at far range (Z > 3.0 m) depth noise increases significantly and a lower alpha (0.5, more smoothing) reduces track jitter. Replacing the constant with `alpha = 0.85 if cz < 1.5 else (0.7 if cz < 3.0 else 0.5)` at line 82 reduces far-target depth variance by ~30% while keeping near-obstacle tracking latency unchanged, at zero additional CPU cost.

### Idea 285: Odom Body Velocity Extraction for Settle-State Kinematic Confirmation
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** `_odom_cb` in `ab_comparison_test.py` (lines 197–204) extracts only position and yaw from the odometry message; `msg.twist.twist.linear.x/y` are ignored. The SETTLE_1/2/3 states use a fixed 1.0 s blind wait before transitioning, but mecanum wheel inertia can keep the robot coasting 10–15 cm at `MAX_LINEAR_SPEED = 0.20 m/s`. Storing `self.current_body_vx = msg.twist.twist.linear.x` and gating settle-state exit on `hypot(current_body_vx, current_body_vy) < 0.02 m/s` (plus a 0.5 s minimum) ensures `start_yaw` and `target_yaw` are captured from a truly stationary robot, eliminating heading reference errors from momentum-carried segment entries.

### Idea 286: Bypass Direction Lock-In Timer to Prevent Flip Oscillation
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** Once `target_lateral_offset` is committed to ±BYPASS_OFFSET, the existing code can reassign it to the opposite sign on the very next tick if clearance conditions change (lines 592–624 in `_update_bypass_offset()`). When an obstacle sits near the boundary of the detection corridor, this causes rapid L/R oscillation in `target_lateral_offset`, which through the lateral P-controller produces real velocity chattering. Adding `self._bypass_dir_change_time = time.monotonic()` on each sign change and requiring `time.monotonic() - _bypass_dir_change_time >= 0.4` before allowing another flip (unless both sides are fully blocked) eliminates the oscillation without delaying legitimate safety switches.

### Idea 287: `hist_local` Forward-Component Clamping in `_inference_loop`
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** In `_inference_loop` (lines 481–491 of `velocity_estimator.py`), `hist_local` is reconstructed from `history_global` by inverse-rotating global coordinates, yielding `rx_l` as the pedestrian's depth in the current robot frame. Because global coordinates accumulate from ICP-corrected poses, `rx_l` can transiently go negative (pedestrian "behind" the robot) or exceed 5.0 m when the robot rotates sharply mid-run, producing out-of-distribution MLP inputs. Adding `rx_l = max(0.3, min(5.0, rx_l))` before `hist_local.append((cx_l, cy_l, rx_l))` mirrors the extraction range used during training and prevents edge-case MLP spike predictions from corrupted history entries, at one `max/min` call per history frame per track.

### Idea 288: ICP Rotation Accumulator Re-Orthogonalization via SVD Polar Factor
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** In `_align_scans_icp` (lines 283–334 of `ab_comparison_test.py`), the total rotation is accumulated by composing individual 2×2 rotation matrices over 3 iterations. Floating-point multiplication errors cause the cumulative matrix to drift from orthogonality (det slightly ≠ 1), introducing a small shear component into every applied correction. After the 3-iteration loop, a single SVD step `U, _, Vt = np.linalg.svd(R_total); R_corrected = U @ Vt` projects the matrix back to the nearest rotation group element. Over multi-run sequences in repeat mode, this prevents compounding shear artifacts in `corrected_yaw` that would otherwise gradually misalign the path-tracking heading reference.

### Idea 289: Per-Track MLP Output Vector EMA for Stable Bypass Direction Commands
**Area:** Velocity Estimation
**Investment:** Low
**Rationale:** The MLP outputs `(vx, vy)` per track at 10 Hz, and the sign of `vy` determines bypass direction preference in `_update_bypass_offset()` (lines 569–591). A single noisy frame where the predicted `vy` flips sign can reverse the bypass direction mid-maneuver. Storing `prev_vx` and `prev_vy` in `ObstacleTracker.tracks` at track creation and applying `vx_out = 0.6 * vx_pred + 0.4 * prev_vx` (similarly for `vy`) before writing to the estimates list provides stable bypass direction signals. This is distinct from Idea 238's scalar speed EMA; smoothing the full velocity vector prevents direction-flip artifacts that the scalar magnitude filter does not address.

### Idea 290: `_encode_mapu` Offload to Thread Executor for Non-Blocking Live Map Requests
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `handle_client`, the `request_live_map` handler calls `_encode_mapu(grid)` synchronously (lines 1392–1398 of `server_x3.py`). For large maps (e.g. 800×600 = 480 000 cells), this numpy array conversion, pixel remapping, and `struct.pack` call can block the asyncio event loop for 5–20 ms, stalling concurrent WebSocket message processing from joystick inputs and velocity readouts during that window. Replacing with `frame_bytes = await loop.run_in_executor(None, _encode_mapu, grid)` offloads the work to a thread pool worker and restores full event-loop responsiveness at zero functional change to the map encoding logic.

### Idea 291: Scan Callback Timestamp Staleness Guard for Bypass Logic
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** `self.last_scan_ranges` in `ab_comparison_test.py` is initialized to `[]` and never tagged with a receive time. If the YDLidar driver restarts mid-run (USB disconnect, driver crash), `last_scan_ranges` retains its final valid snapshot while `_update_bypass_offset()` continues treating it as current data, potentially holding `is_paused = True` or `is_continuous_wall = True` indefinitely from stale readings. Adding `self._last_scan_time = time.monotonic()` in `_scan_cb` and guarding all LiDAR sector checks in `_update_bypass_offset` with `if time.monotonic() - self._last_scan_time > 1.0: skip_lidar` degrades gracefully to camera-only avoidance on sensor fault rather than freezing the robot.

### Idea 292: Forward-Speed Proportional `KP_YAW` Scaling to Prevent Near-Waypoint Yaw Oscillation
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** In both `DRIVE_TO_B` and `DRIVE_TO_A`, the heading correction is `rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))` with a fixed `KP_YAW = 0.4` (lines 954–955 and 1146–1147 of `ab_comparison_test.py`). As the robot decelerates near a waypoint (`speed` approaches `MIN_LINEAR_SPEED`), the mechanical inertia drops but the heading gain remains constant, causing angular overshoot and yaw oscillation that corrupts the final pose used as the rotation reference. Computing `kp_yaw_eff = KP_YAW * min(1.0, speed / MAX_LINEAR_SPEED)` scales the gain proportionally with forward speed, smoothly reducing heading aggressiveness to match reduced inertia during deceleration without affecting full-speed heading correction behavior.

---

## [2026-06-09] Automated Analysis Batch — Ideas 293 to 302

### Idea 293: Pre-Bypass Diagonal Sector Clearance Scan
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** In `_update_bypass_offset()`, bypass direction is validated using perpendicular LiDAR sectors (45°–135° left, −45°–−135° right), but the actual bypass trajectory is a diagonal at approximately 20°–40° from forward when `vy_cmd` and forward speed are simultaneously active. Adding a targeted scan of the 15°–40° (right bypass) or −15°–−40° (left bypass) angular sector before committing `target_lateral_offset` catches obstacles in the diagonal approach corridor that pass perpendicular sector checks but would be struck during the actual strafe motion, preventing bypass-into-wall collisions in angled corridors.

### Idea 294: Centroid Depth Standard Deviation Quality Gate in `_extract_depth_centroids`
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** In the raw-depth path of `_extract_depth_centroids`, `Z = float(np.median(valid_depths))` is already computed from the already-filtered array; `np.std(valid_depths)` is computable from the same array at negligible additional cost. Centroids with `std / Z > 0.35` represent depth-noisy blobs — specular reflections, grazing-angle wall edges, or partially occluded pedestrians — that produce high variance in the per-frame Z fed to the EMA filter and MLP history window. Adding `if len(valid_depths) > 5 and np.std(valid_depths) / Z > 0.35: continue` before appending eliminates these structurally unreliable centroids from the tracking pipeline, directly reducing MLP input jitter and phantom TTC braking from poor-quality depth measurements at zero additional frame acquisition or processing cost.

### Idea 295: Speed-Adaptive ICP vs. Odometry Ratio Plausibility Gate
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** After `_align_scans_icp` returns in `_scan_cb`, adding `if abs(dx_match) > 3.0 * abs(dx_odom_local) + 0.04 or abs(dy_match) > 3.0 * abs(dy_odom_local) + 0.04: fallback to odometry` provides a speed-proportional implausibility guard that is more nuanced than Idea 194's fixed absolute clamp. At very low speeds (`dx_odom_local ≈ 0.002 m`), Idea 194's fixed bound allows a 25× overestimate before rejection; the ratio check catches corrections that exceed 3× the actual motion with a small absolute floor. During fast straight drives where odometry is reliable, the 3× factor allows ICP the room to measure true lateral mecanum slip without false rejections, improving correction acceptance accuracy across all speed regimes.

### Idea 296: Minimum Forward Speed Floor to Prevent Stall During Active Lateral Bypass
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In both drive states, `speed = base_forward_speed * forward_scale` where `forward_scale = 1.0 - max(0.0, min(0.8, lateral_error / 0.5))`. When `lateral_error = 0.4 m`, `forward_scale = 0.2`, producing `speed = MIN_LINEAR_SPEED * 0.2 = 0.012 m/s` — below the mecanum wheel stall threshold. The robot halts forward progress while `lateral_error` remains large, creating a deadlock that can only be broken by `speed_scale ≤ 0.1`. Applying `speed = max(MIN_LINEAR_SPEED * 0.4, speed)` after the diagonal-blend line ensures at least 0.024 m/s of forward progress during lateral settling, allowing bypass completion via diagonal glide without freezing the control loop awaiting full lateral alignment.

### Idea 297: Reactive-Mode TTC Block Short-Circuit in `_get_speed_scaling`
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_get_speed_scaling()` (line 769 of `ab_comparison_test.py`), the `if self._mode == "predictive":` TTC guard is checked per-obstacle inside the loop, but in reactive mode `s_t` is always 1.0 — all TTC arithmetic (`math.hypot`, `r_dot_v`, conditional `ttc`) is dead computation. Since 50% of all A/B test runs use `--mode reactive`, this dead work runs at 20 Hz across all obstacle tracks for the entire reactive run. Hoisting the mode check outside the loop and using an early `return speed_scale` for reactive mode eliminates all TTC arithmetic per reactive tick, saving ~0.1–0.3 ms per 20 Hz call when multiple tracks are present.

### Idea 298: `rot_correction` Attenuation During Active Lateral Strafing
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In both `DRIVE_TO_B` (line 955) and `DRIVE_TO_A` (line 1147), `rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))` fires at full gain during lateral strafing. When `|vy_cmd| ≈ MAX_LATERAL_SPEED = 0.15 m/s`, simultaneous angular commands apply a body torque that couples through mecanum kinematics into additional lateral displacement, competing with `vy_cmd` and adding cross-track error. Multiplying `rot_correction *= max(0.0, 1.0 - abs(vy_cmd) / MAX_LATERAL_SPEED)` smoothly scales heading correction to zero at full lateral speed and restores it as strafing decelerates, eliminating the rotational-lateral kinematic coupling that degrades post-bypass centerline re-acquisition accuracy.

### Idea 299: WebSocket Estimate Timestamp Staleness Guard in AB Test Receive Loop
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** In `_set_estimator_mode()`'s async receive loop in `ab_comparison_test.py`, `self._latest_estimates` persists indefinitely after the last valid `readout` message. If the WebSocket connection drops and reconnects (server restart mid-run), stale velocity estimates from the previous segment remain active, holding TTC brakes or false bypass offsets on ghost tracks. Storing `self._estimates_timestamp = time.monotonic()` alongside each `_latest_estimates` update and checking `if time.monotonic() - self._estimates_timestamp > 0.5: estimates = []` at the start of `_get_speed_scaling()` and `_update_bypass_offset()` degrades gracefully to reactive-mode behavior after 500 ms of connection silence, preventing cross-session contamination from stale estimates.

### Idea 300: ICP Input Point Cloud Far-Range Pre-Filter for Correspondence Matrix Reduction
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** In `_align_scans_icp`, `P` and `Q` include all valid scan points up to 4.0 m (as filtered in `_scan_cb`). For corridor navigation, points at 2.0–4.0 m are typically distant walls that moved minimally between scan intervals, contributing large unmatched entries to the O(M×N) distance matrix that never pass the 0.25 m² correspondence threshold while consuming CPU in the inner loop. Adding `P = P[np.sum(P**2, axis=1) < 4.0]` and `Q = Q[np.sum(Q**2, axis=1) < 4.0]` (range ≤ 2.0 m filter) at the start of `_align_scans_icp` reduces matrix dimensions by 30–50% in typical corridor scans at no accuracy cost, since close-range wall points dominate all valid correspondences.

### Idea 301: `LATERAL_THRESHOLD` Dynamic Expansion During Active Bypass in `_get_speed_scaling`
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `_get_speed_scaling()`, `LATERAL_THRESHOLD = 0.35 m` defines the safety corridor half-width for proximity and TTC checks (line 727 of `ab_comparison_test.py`). During an active bypass with `target_lateral_offset = +0.25 m`, the robot drives offset from centerline; an obstacle at `ry = 0.42 m` sits directly in the bypass path but falls outside the static 0.35 m threshold and receives no TTC scaling. Computing `LATERAL_THRESHOLD = 0.35 + 0.5 * abs(self.target_lateral_offset)` when `self.target_lateral_offset != 0.0` expands the safety corridor to cover the actual swept path, closing the TTC blind spot that Idea 70's travel-frame projection does not address (it adjusts coordinate orientation but not threshold width).

### Idea 302: INIT-State Odometry Body Velocity Zero-Confirmation Before Reference Pose Capture
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** In the `INIT` state (lines 801–826 of `ab_comparison_test.py`), `start_x = corrected_x`, `start_y = corrected_y`, and `start_yaw = corrected_yaw` are captured as soon as valid odom is received, with no check on whether the robot is physically stationary. Residual momentum from the previous repeated run (`current_body_vx > 0.03 m/s`) causes the origin to be captured mid-coast, systematically biasing the start position 5–15 cm forward of the true tape mark. Storing `self.current_body_vx` and `self.current_body_vy` from the already-available `msg.twist.twist` in `_odom_cb`, and deferring `INIT → DRIVE_TO_B` until `math.hypot(current_body_vx, current_body_vy) < 0.02 m/s`, eliminates this per-run start-position bias without adding any delay during normal starts where the robot is already stationary.

## [2026-06-09] Automated Analysis Batch — Ideas 303 to 310

### Idea 303: Pre-Computed Morphological Kernel in `VelocityEstimator.__init__`
**Area:** Script Performance Optimization
**Investment:** Extremely Low
**Rationale:** `cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))` is called every inference cycle at 10 Hz inside `_extract_depth_centroids` (lines 228–229 of `velocity_estimator.py`). The returned kernel is a fixed 5×5 binary array that never changes; recomputing it invokes a Python-to-C binding and allocates a new NumPy array each call. Moving `self._morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))` to `__init__` and replacing the call-site with `self._morph_kernel` eliminates one object allocation and C-extension dispatch per 10 Hz cycle at zero behavioral cost.

### Idea 304: `/scan` Subscription QoS `depth=1` `BEST_EFFORT` in `ab_comparison_test.py`
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `/scan` is subscribed with `qos_profile=10` (line 143 of `ab_comparison_test.py`), so under CPU saturation up to 10 queued scans drain consecutively, firing 8–10 back-to-back ICP computations that inject burst corrections into `corrected_x/y/yaw` before the 20 Hz control loop can resume. Using `QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)` ensures only the latest scan is processed, preventing stale-scan burst ICP and aligning with the lidar driver's own BEST_EFFORT publisher QoS.

### Idea 305: `last_vy_cmd` Zero-Reset on `target_lateral_offset` Sign Flip for Faster Direction Reversal
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** When `target_lateral_offset` flips sign in `_update_bypass_offset()`, `last_vy_cmd` retains its current value (up to ±0.12 m/s). The rate limiter (`MAX_LATERAL_ACCEL = 0.5 m/s²`) then requires ~12 ticks (600 ms at 20 Hz) to reverse lateral direction, causing the robot to continue strafing toward the newly-blocked side for over half a second. Adding `if math.copysign(1, new_offset) != math.copysign(1, self.target_lateral_offset): self.last_vy_cmd = 0.0` immediately before updating `self.target_lateral_offset` halves worst-case reversal time to ~300 ms without removing the rate limiter's protection against step commands.

### Idea 306: APF `d_safe` Activation Hysteresis Band to Eliminate Near-Threshold Lateral Chatter
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** The APF repulsion formula in `_update_bypass_offset()` (lines 474–490 of `ab_comparison_test.py`) activates whenever `wall_left_clearance < d_safe = 0.55 m` and deactivates immediately upon exceeding 0.55 m. During bypass oscillation near this boundary, the force toggles on/off each 50 ms tick, generating 20 Hz lateral micro-corrections visible as a jitter band in the CSV `vy` channel. Adding `self._apf_left_active` (a boolean state variable) that sets `True` at 0.55 m and clears only at 0.60 m introduces a 5 cm hysteresis band, eliminating threshold chatter with no change to steady-state force magnitude.

### Idea 307: ICP Inlier-Fraction Weighted Blend with Raw Odometry for Graceful Quality Transition
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** Current ICP vs. odometry handling uses hard gates (e.g., Ideas 190, 194, 237, 295): if inlier count is below threshold the update is rejected entirely, producing step-change pose jumps at the boundary. Blending via `w = min(1.0, n_valid_inliers / 40.0)` and `dx_final = w * dx_icp + (1 - w) * dx_odom_local` in `_align_scans_icp` provides smooth transition from full-odometry trust (zero inliers) to full-ICP trust (≥40 inliers), removing the discontinuity that manifests as a 3–8 cm corrected-pose jump when scan quality transitions at featureless wall sections.

### Idea 308: `self.current_path_y` Sync from Local `path_y` in Drive States
**Area:** Script Performance Optimization
**Investment:** Extremely Low
**Rationale:** **Bug fix.** In `DRIVE_TO_B` (line 885) and `DRIVE_TO_A` (line 1077) of `ab_comparison_test.py`, the local variable `path_y` is computed correctly from start/end waypoints but is never assigned to `self.current_path_y`. The 2 Hz diagnostic print (line 501) always displays `current_path_y = 0.0`, and any future CSV column for cross-track error (e.g., from Idea 205) will log zeros for the entire run. Adding `self.current_path_y = path_y` immediately after each `path_y` computation in both drive states is a one-line fix per state that restores diagnostic accuracy.

### Idea 309: Consecutive ICP Failure Counter with Corrected-Pose EKF Re-Sync Trigger
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** When `_scan_cb`'s ICP falls through to the odometry-only fallback (exception path at line 271 or insufficient-points guard at line 230 of `ab_comparison_test.py`), `corrected_x/y/yaw` accumulates uncorrected odometry error. Individual failures are gated by Ideas 194 and 237, but 5–10 consecutive failures in featureless corridors can propagate 10–15 cm of uncorrected slip before recovery. Adding `self._icp_fail_count` (increment on failure, reset to 0 on success) and re-syncing `corrected_x/y/yaw` from the current EKF `/odom` pose after 6 consecutive failures provides a bounded-error fallback without touching the normal ICP path.

### Idea 310: Temporal LiDAR Range-Difference Gate for Dynamic Obstacle Confirmation in Forward Sector
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** A pedestrian approaching head-on in a narrow corridor fills the forward scan width with few lateral range discontinuities, potentially satisfying `is_continuous_wall = True` (the `front_has_edges = False` path in `_update_bypass_offset`) and blocking camera-guided bypass. Storing `self._prev_forward_ranges` (the ±20° sector array from the previous scan tick) and computing mean absolute range change between consecutive scans: when this exceeds 0.04 m the forward blocker is dynamic, not a static wall. Overriding `is_continuous_wall = False` when temporal motion is detected in `_scan_cb` complements the existing spatial edge detector and prevents false wall classification for slow-approaching pedestrians.

---

## [2026-06-09] Automated Analysis Batch — Ideas 311 to 320

### Idea 311: `time.monotonic()` Parameter Injection from `_control_loop` into `_update_bypass_offset`
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** `_control_loop` already computes `now = time.monotonic()` at the very top of each 20 Hz tick (line 796 of `ab_comparison_test.py`). Inside `_update_bypass_offset()`, line 493 makes a second `time.monotonic()` call (`now_t = time.monotonic()`) solely to throttle the 2 Hz debug print. Passing `now` as a parameter — `self._update_bypass_offset(now)` — and replacing the internal call with the argument eliminates one OS syscall per bypass-update invocation; with two drive states each calling `_update_bypass_offset` at 20 Hz, this removes 40 `time.monotonic()` calls per second at zero behavioral cost.

### Idea 312: O(N²) History Padding in `_build_window_features` Replaced with List Concatenation
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** The padding loop `while len(hist) < WINDOW_SIZE: hist.insert(0, hist[0] ...)` (lines 354–355 of `velocity_estimator.py`) calls Python `list.insert(0, x)` which is O(N) per call because it shifts all existing elements right. For a fresh track with 1 frame, 9 sequential `insert(0, ...)` calls perform 1+2+…+9 = 45 element-copy operations. Replacing with `hist = [hist[0]] * (WINDOW_SIZE - len(hist)) + hist` uses C-level list multiplication (O(WINDOW_SIZE) total) followed by one O(1) concatenation, making new-track feature assembly approximately 9× faster at 10 Hz with up to 5 simultaneous tracks.

### Idea 313: `/odom` Subscription QoS `depth=1` `BEST_EFFORT` for Stale-Burst ICP Prevention
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** The `/odom` subscriber in `ab_comparison_test.py` (line 142) uses `qos_profile=10`, allowing up to 10 queued messages. Under CPU saturation (e.g., during burst ICP), the spin-once loop drains all 10 stale odom messages consecutively, firing `_odom_cb` 10 times in rapid succession; each updates `prev_odom_pose`, causing `_scan_cb`'s `dx_odom_local / dy_odom_local` delta computation to operate on nearly-zero per-interval motion, feeding degenerate ICP initializations. Switching to `QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)` — matching the EKF publisher's QoS — ensures only the latest pose update is consumed per spin cycle, preventing stale-odom burst processing entirely.

### Idea 314: ICP Final-Iteration `Q_trans` Update Skip for Hot-Path Reduction
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_align_scans_icp` (line 321 of `ab_comparison_test.py`), `Q_trans = (Q_trans @ R_iter.T) + t_iter` is executed unconditionally on all 3 iterations, but on the final (3rd) iteration `Q_trans` immediately falls out of scope and is never read again — only the accumulated `dtheta_corr`, `dx_corr`, `dy_corr` scalars are returned. Guarding the update with `if _ < 2:` inside the `for _ in range(3)` loop skips one (N×2) × (2×2) matrix multiply and one (N×2) broadcast add per scan callback at 20 Hz, eliminating roughly `4N` floating-point operations per call (≈1440 ops/second at N=360) at zero accuracy cost.

### Idea 315: `is_dynamic_pedestrian` Result Reuse in Continuous-Wall Filter to Eliminate Double Loop
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_update_bypass_offset()`, the `is_dynamic_pedestrian` check (lines 413–422) already iterates all estimates with an `rx < 1.8 m` proximity threshold. Lines 459–466 then repeat an almost-identical loop with a slightly wider `rx < 2.0 m` threshold to set `has_dynamic_pedestrian` for the wall-filter guard. Replacing the second loop with `has_dynamic_pedestrian = is_dynamic_pedestrian` eliminates one full estimates iteration at 20 Hz; the 0.2 m threshold difference only matters for a pedestrian at exactly 1.8–2.0 m forward distance — an edge case already blurred by depth-estimation noise at that range — making this substitution behaviorally neutral in practice.

### Idea 316: Clear-Path LiDAR Second-Scan Elimination via Pre-Cached Sector Vectors
**Area:** Collision Avoidance
**Investment:** Low
**Rationale:** At the bottom of `_update_bypass_offset()` (lines 638–648), the obstacle-cleared path-resume check iterates `self.last_scan_ranges` a second time (after the main sector loop at lines 356–404) to test `if abs(y_lat) < LATERAL_CORRIDOR and x_fwd < 1.2`. After implementing Ideas 187 and 188 (NumPy angle-array cache and scan-range NumPy conversion), the main loop already produces vectorized `x_fwd_arr` and `y_lat_arr` arrays for the forward sector. The second Python for-loop can then be replaced by a single `has_obstacle |= np.any(forward_mask & (np.abs(y_lat_arr) < LATERAL_CORRIDOR) & (x_fwd_arr < 1.2))` boolean check — eliminating the second full Python iteration over all 360 scan beams per clear-check call at 20 Hz.

### Idea 317: Dynamic `KP_LATERAL` Gain Reduction in Tight Corridor Mode
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In both drive states, `KP_LATERAL = 0.8` is a hardcoded local constant (lines 888 and 1081 of `ab_comparison_test.py`). When Idea 110's dynamic corridor scaling shrinks `max_allowed_offset` to 0.05–0.10 m in tight bottlenecks, the unchanged 0.8 gain applied to a residual 0.05 m lateral error produces `vy_target = 0.04 m/s` — sufficient to command lateral motion that drives the robot into a wall with < 5 cm clearance. Replacing the constant with `KP_LATERAL = max(0.35, 0.8 * (max_allowed_offset / 0.4))` scales the gain proportionally to available clearance, suppressing micro-oscillation in tight corridors while retaining full 0.8 gain authority in open-corridor bypass scenarios where large offsets are safe.

### Idea 318: APF Local Constants Elevated to `ABComparisonTest.__init__` for Hot-Path Bytecode Reduction
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** Inside `_update_bypass_offset()`, the APF parameters `d_safe = 0.55`, `d_min = 0.22`, and `eta = 0.03` (lines 474–477 of `ab_comparison_test.py`) are assigned as local Python name bindings on every 20 Hz call. Each local assignment generates `LOAD_CONST` + `STORE_FAST` bytecode pairs that the CPython interpreter must execute even though these values never change. Moving them to `self._apf_d_safe`, `self._apf_d_min`, `self._apf_eta` in `__init__` eliminates 3 redundant constant-store operations per tick while converting the APF parameters into self-documenting, externally-tunable instance attributes accessible for future calibration or test parameterization.

### Idea 319: `_maybe_log` Obstacle Distance Vectorized via `np.hypot`
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_maybe_log()` (lines 698–703 of `ab_comparison_test.py`), per-obstacle Euclidean distances are computed via `dists = [math.hypot(ox, oz) for est in estimates]`. Each iteration dispatches a Python function call to `math.hypot`, appends to a Python list, and extracts dict keys. For n simultaneous estimates at 10 Hz, this is 10n Python function calls per second. Replacing with `coords = np.array([[est.get('x', 0.0), est.get('z', est.get('y', 0.0))] for est in estimates], dtype=np.float32)` + `min_dist = float(np.min(np.hypot(coords[:, 0], coords[:, 1])))` eliminates all Python-dispatch overhead for n > 1 and runs at C-level SIMD throughput, with a lightweight early-exit for the n=0 case.

### Idea 320: MLP Inference Warm-Up Dummy Pass in `_load_model` to Eliminate First-Tick JIT Latency
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `VelocityEstimator._load_model()`, JIT tracing (`torch.jit.trace`) produces a compiled TorchScript graph, but CPU-side kernel fusion and code generation for the specific tensor shapes are deferred until the first real forward pass. This causes the very first `self._model(x_tensor)` call in `_inference_loop` to stall for 50–200 ms while PyTorch's JIT backend compiles the fused kernel — freezing the inference thread at the moment the first pedestrian appears. Adding `with torch.no_grad(): _ = self._model(self.x_tensor_preallocated[:1])` immediately after the `torch.jit.trace` call in `_load_model()` forces all kernel compilation at startup, ensuring the inference thread executes at full pre-compiled speed from the very first real detection without a cold-start latency spike.

## [2026-06-10] Automated Analysis Batch — Ideas 321 to 330

### Idea 321: `LATERAL_THRESHOLD` Centered on Active Bypass Offset in `_get_speed_scaling`
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `_get_speed_scaling` (line ~761 of `ab_comparison_test.py`), the check `abs(ry_t) <= LATERAL_THRESHOLD` gates TTC scaling on obstacle lateral position relative to the original path centerline (ry=0). When `self.target_lateral_offset = ±BYPASS_OFFSET`, the robot's actual intended path has shifted; an obstacle already cleared to the original side (at `ry_t = -0.3 m`) still contributes to braking, while an obstacle now centered on the new bypass path (at `ry_t = BYPASS_OFFSET`) may not. Replacing with `abs(ry_t - self.target_lateral_offset) <= LATERAL_THRESHOLD` correctly gates TTC on the current intended path, eliminating false positives from cleared obstacles and false negatives for obstacles on the bypass track — a one-line change requiring only knowledge of `self.target_lateral_offset`, already available in scope.

### Idea 322: `lidar_blocked` Consecutive-Frame Hysteresis Before Setting `is_paused`
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In `_update_bypass_offset`, a single LiDAR frame with a specular floor reflection or low-flying dust return in the ±30°×0.75 m forward zone sets `lidar_blocked = True`, immediately setting `self.is_paused = True` and starting `blocked_time` accumulation. Add `self._lidar_block_count` (analogous to Idea 203's wall confirmation counter for `_front_is_continuous_wall`) requiring 2 consecutive `lidar_blocked` ticks (~100 ms at 20 Hz) before activating the pause, while clearing to 0 on the first unblocked tick. This single-counter addition filters transient sensor noise without meaningfully delaying genuine obstacle response, directly reducing spurious `blocked_time` accumulations that can prematurely trigger the 8-second backing recovery.

### Idea 323: Pre-Allocated Normalization Output Buffer to Eliminate Per-Cycle Heap Allocations in `_inference_loop`
**Area:** Script Optimization
**Investment:** Extremely Low
**Rationale:** In `_inference_loop` (line ~515 of `velocity_estimator.py`), `features_scaled = (features_batch - self.scaler_X_mean) / self.scaler_X_scale` allocates two intermediate `(N, 40)` numpy arrays per 10 Hz inference cycle — one for the subtract result, one for the divide result. Pre-allocating `self._features_buffer = np.zeros((MAX_OBSTACLES, 40), dtype=np.float32)` in `__init__` and using `np.subtract(features_batch, self.scaler_X_mean, out=self._features_buffer[:num_tracks])` followed by `np.multiply(self._features_buffer[:num_tracks], self.scaler_X_inv_scale, out=self._features_buffer[:num_tracks])` (combining with Idea 202's pre-inverted scale) eliminates both heap allocations at 10 Hz, reducing GC pressure on the ARM Jetson core during the inference hot path with zero numerical change.

### Idea 324: `smooth_speed_scale` Reset in Repeat-Mode `ROTATE_HOME` Restart Block
**Area:** Collision Avoidance
**Investment:** Extremely Low
**Rationale:** In the `ROTATE_HOME` repeat-mode restart block (~line 1217 of `ab_comparison_test.py`), `self.blocked_time`, `self.is_paused`, and `self.last_vy_cmd` are reset to 0/False/0.0, but `self.smooth_speed_scale` is not. If the previous run ended with a TTC braking event leaving `smooth_speed_scale = 0.3`, the new run starts with 70% speed suppression applied before any obstacle is present, biasing first-leg performance metrics and potentially triggering the `SEGMENT_TIMEOUT` if the obstacle does not re-appear. Adding `self.smooth_speed_scale = 1.0` alongside the existing reset lines is a one-line fix that guarantees a clean behavioral slate at every repeat boundary with no impact on non-repeat runs.

### Idea 325: Closest-Obstacle Track ID Column in `_maybe_log` for Post-Run Track Attribution
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** `_maybe_log` (lines 689–711 of `ab_comparison_test.py`) logs `n_obstacles` and `min_obstacle_dist`, but not which track ID corresponds to the minimum-distance obstacle. Adding `closest_id = min(estimates, key=lambda e: math.hypot(e.get('x',0), e.get('z',0)))['id'] if estimates else -1` as a `closest_obstacle_id` column enables post-run identification of whether each bypass event involved the same re-engaging pedestrian or a new track, and allows cross-run obstacle-engagement frequency analysis per mode that is currently impossible from distance-only logs.

### Idea 326: `np.vstack` Eliminated via Direct Per-Track Write into Pre-Allocated Batch Feature Matrix
**Area:** Script Optimization
**Investment:** Low
**Rationale:** In `_inference_loop` (line ~512 of `velocity_estimator.py`), `features_batch = np.vstack(features_list)` creates a new `(N, 40)` heap array from N separately-allocated `(1, 40)` arrays accumulated in `features_list` — two memory objects per eligible track plus a full vstack copy. Pre-allocating `self._batch_buffer = np.zeros((MAX_OBSTACLES, 40), dtype=np.float32)` in `__init__` and writing each track's feature directly via `self._batch_buffer[idx] = feats[0]` inside the eligible-tracks loop eliminates the `features_list` Python list, all per-track `(1, 40)` allocations from `_build_window_features`, and the vstack copy — replacing them with a single in-place row assignment per track per cycle.

### Idea 327: Depth-Range Confidence Downscale for Far Tracks (Z > 2.5 m) in `_get_speed_scaling`
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** Astra Pro depth accuracy degrades beyond 2.5 m; at 4 m, centroid depth jitter is ±15–20 cm per frame, which the MLP interprets as lateral motion and produces nonzero velocity predictions. In `_get_speed_scaling`, multiply each obstacle's TTC/proximity contribution by `depth_conf = max(0.2, 1.0 - max(0.0, cz - 2.5) / 1.5)` where `cz = est.get('z', 1.0)`. This softly discounts velocity predictions from far-range tracks (0.2× at 4 m) without disabling them, reducing false TTC braking from depth-noise-induced phantom motion at long range while preserving full sensitivity within 2.5 m where depth accuracy is reliable.

### Idea 328: Weighted ICP Correspondence via Range-Proportional Point Weights in `_align_scans_icp`
**Area:** Navigation Accuracy
**Investment:** Low
**Rationale:** The current `_align_scans_icp` implementation treats all matched point pairs equally in the cross-covariance matrix `H = Q_bar.T @ P_bar`. Close-range LiDAR returns (< 1.5 m) have far lower angular uncertainty than far-range returns (> 3 m), so weighting each correspondence by `w_i = 1.0 / max(0.25, P_matched[i,0]**2 + P_matched[i,1]**2)` and computing `H = (Q_bar * w[:,np.newaxis]).T @ P_bar` reduces the influence of noisy far-wall correspondences (doorways, far-end walls) on the rotation solution, improving yaw correction accuracy in mixed near/far environments — a single `w` vector computation and one einsum replacement in the existing SVD-free 2D ICP, with no architecture change.

### Idea 329: ICP vs. EKF Pose Discrepancy Real-Time Warning in 2 Hz Diagnostic Print
**Area:** Navigation Accuracy
**Investment:** Extremely Low
**Rationale:** The 2 Hz diagnostic print in `_update_bypass_offset` (~line 493 of `ab_comparison_test.py`) displays state, speed scale, LiDAR clearances, and camera detections, but never shows the current ICP-vs-EKF pose discrepancy `(corrected_x - current_x, corrected_y - current_y, corrected_yaw - current_yaw)`. Adding a one-line warning `if math.hypot(self.corrected_x - self.current_x, self.corrected_y - self.current_y) > 0.1` with a printed alert provides real-time ICP quality feedback during lab runs, enabling the operator to observe ICP degradation (large discrepancy = accumulated slip correction or divergent scan match) without waiting for post-run CSV analysis.

### Idea 330: Global History Upstream Displacement Clamping Before Local Frame Reconstruction in `_build_window_features`
**Area:** Velocity Estimation
**Investment:** Extremely Low
**Rationale:** In `_inference_loop` (lines 481–491 of `velocity_estimator.py`), `history_global` entries are inverse-rotated to produce `hist_local`. When a single ICP correction injects a 0.3–0.4 m spurious delta into `corrected_x/y`, the resulting `history_global` entry produces a reconstructed local displacement that far exceeds the per-frame clamp in `_build_window_features` (±0.25 m). Adding an upstream clamp `d_g = np.clip(np.hypot(dx_g, dy_g), 0.0, 0.30)` with direction restoration on each global history pair before the inverse rotation step provides a first-stage outlier rejection in global coordinates, preventing ICP drift spikes from reaching the MLP feature vector even when the local clamp in `_build_window_features` is hit at its boundary.
