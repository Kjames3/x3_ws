# Strafe live test: omni-direct MPPI + keep-yaw goals (2026-11-26)

Background: `notes/plans/overnight_perf_2026-11-26.md` section 4 and
`artifacts/overnight-perf-2026-11-26/strafe_diagnosis_sonnet.md`.

Both changes are **opt-in, and the default is unchanged**:

| Piece | Switch | Default |
|---|---|---|
| `src/yahboomcar_nav/params/nav2_params_x3_omni_direct.yaml` (plain MPPI, no RotationShim) | `X3_NAV2_PARAMS=omni_direct` in x3_server's environment | unset, so `nav2_params_x3.yaml` is used |
| GUI plain click keeps the current map heading instead of yaw 0 | `http://orin:8080/GUI.html?navKeepYaw=1` (remembered in localStorage; `?navKeepYaw=0` turns it off) | off |

**Nothing here has run on the robot.** Every "expect" below is a prediction from the
diagnosis, not something measured.

All robot commands run as `jetson@orin` with ROS sourced:

```bash
source /opt/ros/humble/setup.bash && source ~/x3_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_DISCOVERY_SERVER=127.0.0.1:11811
export FASTDDS_DEFAULT_PROFILES_FILE=~/x3_ws/config/fastdds_tcp_server.xml   # match the unit
```

## 0. Deploy and build

Get this branch onto the robot first, then rebuild yahboomcar_nav. Nav2 loads params from
the **install** tree, so a new yaml does not exist for Nav2 until it is built. That holds
even with `--symlink-install`, because the symlink for a new file is only created by a build.

First check how the robot's install tree was built. Whether it uses symlinks is the
open question in the diagnosis.

```bash
cd ~/x3_ws
ls -l install/yahboomcar_nav/share/yahboomcar_nav/params/nav2_params_x3.yaml
#   "-> .../src/..."  symlink install:  colcon build --symlink-install --packages-select yahboomcar_nav
#   regular file      plain install:    colcon build --packages-select yahboomcar_nav
#                     (and every later yaml edit needs this rebuild too)
ls -l install/yahboomcar_nav/share/yahboomcar_nav/params/nav2_params_x3*.yaml   # after the build
```

### Stale-install check (do this before any driving)

The diagnosis found the laptop's installed `nav2_params_x3.yaml` still on `DiffDrive`
with no smoother block. A stale copy alone reproduces the bug.

```bash
cd ~/x3_ws
for f in nav2_params_x3.yaml nav2_params_x3_omni_direct.yaml; do
  sha256sum src/yahboomcar_nav/params/$f install/yahboomcar_nav/share/yahboomcar_nav/params/$f
done
# each pair must match (or the install entry is a symlink to src)
```

## 1. Enable omni-direct

A variable exported in an SSH shell **does not reach the systemd unit**. Add it to the
existing override instead, and keep whatever lines are already there:

```bash
sudo systemctl edit x3_server
#   [Service]
#   Environment=X3_NAV2_PARAMS=omni_direct
sudo systemctl daemon-reload && sudo systemctl restart x3_server
```

(Or run the server by hand: `X3_NAV2_PARAMS=omni_direct python3 src/server_x3.py --domain-id 42`.)

Nav2 is a long-lived subprocess. If it was already running, **stop Nav2 and launch it
again from the GUI**, or the old params stay loaded. Then confirm which params file
was picked:

```bash
journalctl -u x3_server --since "5 min ago" | grep "Nav2 params file"
#   expect: [Nav2Client] Nav2 params file: /home/jetson/x3_ws/install/.../nav2_params_x3_omni_direct.yaml (from X3_NAV2_PARAMS='omni_direct')
#   "ERROR: ... does not exist" means the build in step 0 was skipped; the server fell back to the default
journalctl -u x3_server --since "5 min ago" | grep "Launching Nav2"
#   omni: the command line ends with params_file:=.../nav2_params_x3_omni_direct.yaml
#   default: no params_file:= argument at all
# (~/.ros/log/x3_nav2_launch.log has the Nav2 console output, for load errors)
```

## 2. Param checks on the live nodes

```bash
ros2 param get /controller_server FollowPath.plugin
#   omni:    nav2_mppi_controller::MPPIController
#   default: nav2_rotation_shim_controller::RotationShimController
ros2 param get /controller_server FollowPath.primary_controller    # omni: "Parameter not set"
ros2 param get /controller_server FollowPath.motion_model          # Omni
ros2 param get /controller_server FollowPath.vy_max                # 0.26
ros2 param get /velocity_smoother max_velocity                     # [0.26, 0.26, 1.0]
ros2 param get /velocity_smoother min_velocity                     # [-0.26, -0.26, -1.0]
ros2 topic info -v /cmd_vel_nav   # publisher controller_server, subscriber velocity_smoother
```

If `motion_model` shows `DiffDrive`, or `max_velocity` has 0 in the y slot, the
install is stale (see step 0). Fix that before going on.

## 3. Recording the velocity chain

Use four terminals, or one `ros2 bag record` running alongside:

```bash
ros2 topic echo /plan --field poses --once | head -40      # path direction after each goal
ros2 topic echo /cmd_vel_nav     # controller output (before the smoother)
ros2 topic echo /cmd_vel         # after velocity_smoother
ros2 topic echo /cmd_vel_safe    # after the CBF gate (what the driver gets)
# or:
ros2 bag record -o strafe_$(date +%H%M) /plan /cmd_vel_nav /cmd_vel /cmd_vel_safe /odom /amcl_pose
```

## 4. Test A: side goal, 0.8 m left

- Localize first (AMCL converged, robot marker on the map in the right place). Leave
  at least 0.5 m of clear floor to the left and keep the robot away from walls, because
  the CBF can redirect near obstacles.
- **Hard-reload** the GUI (Ctrl+Shift+R). `GUI.html` loads `main.js?v=nav-keep-yaw-1`;
  if a click never prints a `navKeepYaw:` console line, the old JS is still cached.
- Open the GUI with `?navKeepYaw=1`. In navigate mode, **plain-click** about 0.8 m to the
  robot's left. The browser console should print `navKeepYaw: goal yaw = current map heading N°`.
  Without the toggle, drag along the robot's current heading. A yaw-0 goal muddies the result
  with a final spin.
- Run it once on the **default** params, then once on **omni_direct**.

| | Default (shim), expected | omni_direct, PASS if |
|---|---|---|
| `/cmd_vel_nav` first ~1.5 s | `linear.x = linear.y = 0`, `angular.z ≈ ±1.0` (shim rotate-in-place) | \|linear.y\| reaches 0.2–0.26 within ~1 s, \|angular.z\| < ~0.3 |
| `/cmd_vel`, `/cmd_vel_safe` | follow /cmd_vel_nav | sustained \|linear.y\| 0.2–0.26 on both |
| Final heading change | ~90° | < ~15° (0.25 rad yaw tolerance) |
| Time to goal | — | < ~10 s, no recovery behaviour |

**FAIL / triage:**
- y on `/cmd_vel_nav` but not on `/cmd_vel` points at the smoother.
- y on `/cmd_vel` but not on `/cmd_vel_safe` points at the CBF.
- No y on `/cmd_vel_nav` with `FollowPath.plugin` = MPPI points at the MPPI critics.
  TwirlingCritic is the next knob, and it is not in this patch.
- Hovering near 0 m/s for more than 5 s reproduces the 2026-10-22 0.30 m sideways stall.
  Record it, then retry with a 0.3 m side goal.

## 5. Test B (regression): goal behind the robot

The shim exists because of this case. Under DiffDrive (2026-10-22), plain MPPI sat at
~0 m/s for ~18 s on a goal behind the robot, and 35 s at 1.3 m.

- Goal 1.0–1.3 m straight behind, plain click with keep-yaw, on **omni_direct**.
- **PASS:** the robot starts moving within ~3 s (reverse, strafe, or turn are all fine)
  and reaches the goal with no progress-checker abort (`movement_time_allowance` 20 s).
- **FAIL:** ~0 m/s for more than 5 s, or an ABORTED "failed to make progress". In that case
  omni_direct is not adoptable as it stands. Return to the default file and try the
  diagnosis's fallback: keep the shim with `angular_dist_threshold` ~2.36 and a matching
  lookahead.
- Repeat once with a yaw-0 (no-toggle) goal behind the robot, since that combines rotation
  and translation.

## 6. Rollback

```bash
sudo systemctl edit x3_server      # delete the X3_NAV2_PARAMS line
sudo systemctl daemon-reload && sudo systemctl restart x3_server   # then relaunch Nav2 from the GUI
```

GUI: open once with `?navKeepYaw=0`.

## Limits

- Only Nav2 launched by `server_x3.py` (`Nav2Client.launch_nav2`) honours `X3_NAV2_PARAMS`.
  A manual `ros2 launch yahboomcar_nav x3_nav2.launch.py` needs
  `params_file:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/params/nav2_params_x3_omni_direct.yaml`.
  `x3_localization.launch.py` has its own default and is unaffected.
- The omni yaml is a copy. `tests/test_nav2_omni_direct_params.py` fails if the base file
  changes outside `FollowPath` without the same change in the copy.
