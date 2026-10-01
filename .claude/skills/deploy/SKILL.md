---
name: deploy
description: Commit, push, update the robot's checkout and restart x3_server safely, then confirm the robot is actually working. Use when the user says "commit and deploy", "push this to the robot", "restart the robot/server", or after a code change that must run on the robot.
---

# Deploying to the robot

The robot runs `/home/jetson/x3_ws` on `main` under the `x3_server` systemd
unit. Deploy through git, not scp: copying files over is what makes the two
checkouts drift (see the `robot-sync` skill).

## Steps

1. **Tests on the laptop.** `python3 -m pytest tests -q`. CI runs the same
   suite plus shellcheck and the web checks on push.
2. **Commit and push** from the laptop, in logical commits. The user works on
   `main` directly.
3. **Check the robot is clean enough to pull.**
   `ssh x3 'cd ~/x3_ws && git status --short | grep -v "^??"'`
   The `ydlidar_ros2_driver` submodule line (` m`) is expected. Anything else:
   stop and use `robot-sync` first.
4. **Pull.** `ssh x3 'cd ~/x3_ws && git pull --ff-only'`.
   Never `git reset --hard` there: `install/` is untracked and it deletes
   `install/setup.bash`. If the pull is refused, do not force it.
5. **Rebuild only if a ROS package changed** (anything under
   `src/yahboom*`, the URDF, launch files, params, messages). `install/` holds
   COPIES of params and launch files, so an edit to `src/.../params/*.yaml`
   does nothing until `colcon build`. Never add `--symlink-install`.
   Pure `src/*.py` and `src/web/*` changes need no build.
6. **Restart, if the change needs it.** Ask first if the robot is powered and
   someone may be driving. Web files (`src/web/*`) need only a browser refresh.
   ```bash
   ssh x3 'sudo -n -- /usr/bin/systemctl restart x3_server'
   ```
   This is passwordless. The unit sleeps 10 s before starting and then brings
   up the camera and ROS; wait for the log line, not a fixed sleep:
   ```bash
   ssh x3 'journalctl -u x3_server --since "-2min" --no-pager | grep "Server started on ws"'
   ```
   The `jetson-mcp` tool `restart_and_wait` does both.
7. **Confirm data is flowing.** About one start in three comes up with ROS
   discovery fine and zero messages delivered. The tell is that `x3_server`
   itself receives nothing (lidar toggled on in the GUI shows no points).
   Fix: restart `x3_server`, then `foxglove_bridge`, in that order.
   `foxglove_bridge` is NOT in the passwordless list, so hand the user:
   `sudo systemctl restart foxglove_bridge`.
8. **Report** the robot's new HEAD and what was and was not restarted.

## Changing server flags

Flags live in drop-ins under `/etc/systemd/system/x3_server.service.d/`
(`SERVER_ARGS=`). The highest-numbered file that sets `SERVER_ARGS` wins.
Editing them needs a password sudo, so give the user the command, followed by
`sudo systemctl daemon-reload` and a restart.

## Traps

- Repeated restarts can leave two `Mcnamu_driver_X3` processes fighting over
  the serial port: robot connected but will not move. A power cycle clears it.
- `OLED init failed 0x3C` at boot is expected; the OLED is unplugged.
- Detection and depth toggles reset to OFF on every restart.
- Sourcing only `/opt/ros/humble` gives empty topic lists. Use the preamble in
  the `x3-robot` skill.
