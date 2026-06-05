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
