---
title: Build and Deploy
description: Runbook — build the workspace, sync to the Jetson, restart the service
tags: [runbook]
---

# Runbook: Build and Deploy

Back to [[Home]]. Background: [[project_robot_deploy]].

## Build locally

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select yahboomcar_msgs yahboomcar_description \
  yahboomcar_base_node yahboomcar_bringup yahboomcar_nav ydlidar_ros2_driver
```

Full build including the YDLidar SDK: `bash scripts/build_ros2.sh`.

Tests:

```bash
source install/setup.bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## Reach the robot

Host is `jetson`, IP is DHCP and **changes** — confirm the current one before assuming.
Key-based SSH works: `ssh jetson@<ip>`. `sudo` needs a password.

## Sync code — the trailing-slash trap

The `jetson-mcp` `sync_code` tool runs `rsync <local_path> <remote_path>` with **no trailing
slash** on the local path, so it copies the directory *into* the remote path.

> [!warning] Pass the PARENT directory as `remote_path`
> Correct: `local_path=/home/kamren/x3_ws/src/yahboomcar_description`, `remote_path=/home/jetson/x3_ws/src`
> Wrong: `remote_path=.../src/yahboomcar_description` — creates
> `yahboomcar_description/yahboomcar_description/`, leaves the real package stale, and colcon
> happily builds the old files.

`rsync` uses `--delete`, scoped to the transferred directory only — siblings are untouched.
After syncing, run `colcon_build` with `packages=<pkg>` and **verify the `install/share` copy
actually changed**.

## Restart the service

```bash
sudo systemctl restart x3_server
journalctl -u x3_server -f
```

There's a ~10 s `ExecStartPre` sleep plus camera/ROS init — allow **40–90 s** and poll
`systemctl show -p MainPID`.

> [!danger] Restart is fragile
> Shutdown is unclean: `motion_loop` keeps calling `drive.move()` → `_cmd_vel_pub.publish()`
> while rclpy tears down, throwing `RCLError: publisher's context is invalid` and then a C++
> abort. Repeated restarts can orphan the relaunched bringup — two `Mcnamu_driver_X3`
> processes fighting over `/dev/ttyCH341USB0` — and desync the Rosmaster MCU serial, leaving
> the robot "connected but won't move".
>
> **A full power cycle clears it.** Latent fix worth doing: stop `motion_loop` publishing
> before rclpy teardown, and kill the bringup subprocess cleanly on shutdown.

## Git divergence

The robot's checkout at `/home/jetson/x3_ws` is on `main` but has **diverged** from the local
repo — different commits. A plain `git pull` will not cleanly carry local edits, so fixes
usually have to be applied to both copies. Don't assume the robot is running what you're reading.

## Related

- [[Bringup]], [[Troubleshooting-DDS]], [[project_robot_deploy]]
