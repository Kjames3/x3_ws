# Why the X3 still won't strafe to a side goal after de7df33

Read-only research on branch overnight/perf-2026-11-26 (de7df33 is on main too). Nothing here was run on the robot.

## Ranked root causes

### 1. RotationShimController turns the robot toward the path before MPPI gets a chance (HIGH confidence)
- `src/yahboomcar_nav/params/nav2_params_x3.yaml:159-167`: `FollowPath.plugin: nav2_rotation_shim_controller::RotationShimController`, `primary_controller: MPPI`, `angular_dist_threshold: 0.785` (45 deg), `forward_sampling_distance: 0.15`.
- Humble's shim works like this (recalled from the Nav2 1.1.x source; nav2_rotation_shim_controller isn't installed on this laptop, so I couldn't grep it). `setPlan()` sets `path_updated_ = true`. While that flag is set, `computeVelocityCommands()` takes the path point 0.15 m ahead and finds its bearing in base_link. If |bearing| > threshold, it sends a **pure rotation** (linear x = y = 0) at `rotate_to_heading_angular_vel`. Only once the robot is within the threshold does it clear the flag and hand control to MPPI.
- For a goal directly to the side, the bearing is about 90 deg, so the shim always rotates first. When MPPI takes over, the path is straight ahead and MPPI drives forward. That matches the symptom ("rotated / drove forward instead").
- The BT is the stock `navigate_to_pose_w_replanning_and_recovery.xml` (yaml:74). It replans at 1 Hz, and every new path calls `setPlan`, which re-arms the shim. Any drift past 45 deg sends it back to turn-in-place.
- The shim was added in 4770360 (2026-10-22). DiffDrive was already active then (27dfd3c, 2026-10-21), and under DiffDrive a sideways goal really can't be reached without turning. de7df33 switched MPPI to Omni but left the shim in front of it, so the Omni change never gets used for any goal more than 45 deg off the heading.

### 2. A plain click sends goal yaw = 0 in the map frame, so the robot has to turn at the end anyway (MEDIUM-HIGH)
- `src/web/main.js:796-802`: `let theta = 0;` is only replaced by a drag direction when the drag is at least DRAG_MIN_PX. `server_x3.py:4438` → `nav2_client.navigate_to(x, y, theta)` (`nav2_client.py:161-173`) sends that yaw unchanged.
- `general_goal_checker.yaw_goal_tolerance: 0.25` (yaml:151) and `GoalAngleCritic` (weight 3, `threshold_to_consider: 0.4`, yaml:224-228) then require the robot to finish at map heading 0. Unless the robot already faces map +x, it has to rotate near the goal, so even a perfect strafe ends with a spin. For an apples-to-apples strafe test, the goal yaw has to equal the robot's current heading.

### 3. MPPI Omni has no incentive to hold heading (LOW-MEDIUM, partly speculative)
- Critics (yaml:202-210): Constraint, Cost, Goal, GoalAngle, PathAlign (`use_path_orientations: false`), PathFollow, PreferForward, DBaS. No TwirlingCritic. `wz_std: 0.4` against `vy_std: 0.2`.
- PreferForwardCritic penalizes only vx < 0, not vy (it's a vx-only term in 1.1.x), so it doesn't directly block strafing. But nothing penalizes yaw rate either, so sampled trajectories that turn and drive cost about the same as strafes, and MPPI can freely mix rotation into a sideways move. I haven't verified this live. It only matters once cause 1 is removed.
- `ax_max`, `ay_max` and `az_max` (yaml:180-183) don't appear as strings in `/opt/ros/humble/lib/libmppi_controller.so` (1.1.20), while `vy_max`, `vy_std`, `vx_min`, `wz_max` and `motion_model` all do. Those three keys are probably ignored silently on Humble. That's harmless, but don't rely on them.

### 4. Deployment / staleness (UNVERIFIED, check first)
- Nav2 reads the params from `get_package_share_directory('yahboomcar_nav')` (`x3_nav2.launch.py:43`, `nav2_client.py:283-284`), which is the **install** tree (`setup.py:17` copies `params/*.*`). Unless the robot builds with `--symlink-install` (scripts/install.sh does; the CLAUDE.md build command doesn't), de7df33 has no effect until `colcon build --packages-select yahboomcar_nav` is run.
- Nav2 is a long-lived subprocess, so it also has to be stopped and restarted from the GUI before the new yaml is loaded.

## Ruled out (with evidence)
- **Param namespace**: the shim passes its own plugin name to the primary controller, so MPPI reads `FollowPath.*`. The keys `motion_model: "Omni"`, `vy_max: 0.26` and `vy_std: 0.2` are spelled correctly and present in the 1.1.20 library.
- **velocity_smoother**: the de7df33 block (yaml:266-279) sets y limits of ±0.26 and accel ±2.5, uses OPEN_LOOP, and has a zero deadband. Humble's `nav2_bringup/navigation_launch.py` (from memory of the 1.1.x source; nav2_bringup isn't installed on this laptop) remaps controller `cmd_vel → cmd_vel_nav`, smoother `cmd_vel → cmd_vel_nav` and `cmd_vel_smoothed → cmd_vel`, and lists `velocity_smoother` in lifecycle_nodes. x3_nav2.launch.py includes that file unchanged (lines 84-106). **Check on the robot**: `ros2 param get /velocity_smoother max_velocity`.
- **CBF gate**: `server_x3.py:1919-1933` `_gate_cmd_vel_cb` → `_move_impl(linear.x, linear.y, angular.z)`. `filter_twist` / `_limit_for_deadband` (`server_x3.py:1862-1915`) work on the 2D (vx, vy) vector and only scale along its direction, never zeroing vy alone. The output sets `msg.linear.y = safe_vy * LINEAR_SCALE` (`server_x3.py:2126`).
- **Driver**: `Mcnamu_driver_X3.py:270, 306-309` uses vy in the mecanum mix. Heading hold (lines 150-156, 272-279) adds only an omega correction while translating.
- **Planner**: SmacPlanner2D has no orientation constraints, and `use_final_approach_orientation: false` (yaml:309). Its path headings point along the path, which is what the shim reacts to (cause 1).

## Proposed diff

```diff
--- a/src/yahboomcar_nav/params/nav2_params_x3.yaml
+++ b/src/yahboomcar_nav/params/nav2_params_x3.yaml
@@ -158,7 +158,12 @@ controller_server:
       # recorded 2026-10-22).  MPPI's own parameters below stay in this namespace.
       plugin: "nav2_rotation_shim_controller::RotationShimController"
       primary_controller: "nav2_mppi_controller::MPPIController"
-      angular_dist_threshold: 0.785      # 45 deg
+      # 135 deg (was 45, chosen under DiffDrive): with Omni MPPI strafes or reverses to
+      # anything up to ~135 deg off the heading; the shim only turns the robot for goals
+      # well behind it.  At 45 deg it turned the robot toward every side goal, and the
+      # 1 Hz replan re-armed it (2026-11-26).
+      angular_dist_threshold: 2.36
       # The shim only rotates when the path point this far ahead is >45 deg off the
@@ -207,6 +212,7 @@ controller_server:
         - "PathFollowCritic"
         - "PreferForwardCritic"
+        - "TwirlingCritic"
         - "DBaSCritic"
@@ -233,6 +239,11 @@ controller_server:
         threshold_to_consider: 0.4
+      TwirlingCritic:
+        # Omni: penalise yaw rate so a side goal is strafed, not turned toward.
+        enabled: true
+        cost_power: 1
+        cost_weight: 10.0
       CostCritic:
```

If the 2.36 threshold still triggers rotations, the alternative is to drop the shim entirely: `plugin: "nav2_mppi_controller::MPPIController"` and delete `primary_controller` and the shim keys. But the 2026-10-22 hover-at-~0 m/s notes suggest MPPI alone may stall on goals behind the robot, so try the threshold change first.

GUI goal yaw (cause 2): when the user clicks without dragging, send the robot's current map heading instead of 0:

```diff
--- a/src/web/main.js
+++ b/src/web/main.js
@@ -796,7 +796,9 @@
-        let theta = 0;
+        // No drag: keep the current heading (was 0 = map +x, which forced a final spin).
+        let theta = (state.nav && state.nav.robotPose && Number.isFinite(state.nav.robotPose.theta))
+            ? state.nav.robotPose.theta : 0;
```

(I haven't checked that `state.nav.robotPose.theta` exists. Use whichever field main.js already uses to draw the robot's map pose.)

## Live test checklist
1. Confirm deployment: `ros2 param get /controller_server FollowPath.motion_model` → Omni; `... FollowPath.angular_dist_threshold`; `ros2 param get /velocity_smoother max_velocity` → [0.26, 0.26, 1.0]. If these show old values, rebuild yahboomcar_nav (or use symlink-install) and restart Nav2.
2. `ros2 node list | grep velocity_smoother` and `ros2 topic info -v /cmd_vel_nav`: publisher should be controller_server, subscriber velocity_smoother.
3. Put a goal about 0.8 m to the robot's left, **dragging in the robot's current heading** (or using the GUI fix). Echo `/cmd_vel_nav`, `/cmd_vel` and `/cmd_vel_safe` side by side:
   - With the current config, I expect the first ~1.5 s to show `linear.x = linear.y = 0`, `angular.z ≈ ±1.0` on /cmd_vel_nav. That would confirm the shim.
   - After the fix: |linear.y| ≈ 0.2-0.26 and |angular.z| small on all three topics. If y is present on /cmd_vel_nav but missing on /cmd_vel, the smoother is the problem. If it's present on /cmd_vel but missing on /cmd_vel_safe, the CBF is.
4. Repeat with a goal behind the robot to check that the shim still turns it instead of hovering, the issue noted 2026-10-22.
