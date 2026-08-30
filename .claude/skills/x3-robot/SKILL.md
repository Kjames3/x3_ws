---
name: x3-robot
description: Working knowledge for the X3 Jetson robot - how to reach it, run ROS 2 commands that actually return results, restart its services, check its hardware, and keep the laptop and robot copies of the code in sync. Use whenever a task touches the robot, its services (x3_server, x3_dynamixel_tilt_home), its ROS 2 graph, its serial devices, or its web GUI.
---

# The X3 robot

A Yahboom X3 on a Jetson, driven from this laptop. The `jetson-mcp` MCP server
is configured for this project and its tools already encode everything below --
prefer them over hand-rolled `ssh`. This file is for when you need to shell out
anyway, or need to understand what a tool is doing.

## Reaching it

| | |
| --- | --- |
| Host | `x3` on the LAN, `100.64.52.55` over Tailscale |
| User | `jetson` |
| Laptop workspace | `/home/kamren/x3_ws` |
| Robot workspace | `/home/jetson/x3_ws` (`~/x3_ws`) |

If `ssh` returns **exit 255** or "connection timed out during banner exchange",
the robot is not down -- you are on the wrong address. Try the other one.
`check_connection` and every other MCP tool fall back automatically.

## ROS 2 commands

**Sourcing only the distro returns an empty node and topic list.** Commands
appear to succeed while reporting nothing. Every ROS command needs the full
preamble:

```bash
source /opt/ros/humble/setup.bash
source ~/x3_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
unset ROS_DISCOVERY_SERVER
```

`ROS_DOMAIN_ID=42` is not the default and is required. `ROS_DISCOVERY_SERVER`
is cleared because a stale value in the login shell sends discovery to a server
that is not running.

The MCP ROS tools (`check_ros_status`, `check_ros_node`, `view_ros_topic`,
`check_rosbag_*`) apply this for you.

## Services

| Unit | Enabled | What it is |
| --- | --- | --- |
| `x3_server` | yes | The main server. Web GUI on **8080**, websocket on **8081**. |
| `x3_dynamixel_tilt_home` | yes | Homes the XL430 lidar tilt servo to 2048 at boot. |
| `x3_lidar_home` | no | Superseded by the Dynamixel unit above. |

**Restarting needs root, and `sudo` over SSH has no tty** -- a bare
`sudo systemctl restart` fails with `a terminal is required to read the
password`. `/etc/sudoers.d/x3-mcp` on the robot grants passwordless sudo for
exactly `systemctl {start,stop,restart,enable,disable}` on the units above, so:

```bash
sudo -n -- /usr/bin/systemctl restart x3_server    # works, no password
```

Nothing else is passwordless; `sudo -n` on anything outside that list fails with
`a password is required`. Reading state (`is-active`, `is-enabled`, `journalctl`)
needs no sudo at all -- the `jetson` user is in `adm`.

Do not try to route a privileged command through `sudo bash -c '...'`; the
sudoers rule does not authorize a shell. Run the `systemctl` call on its own and
do the surrounding work unprivileged. `manage_service` and `restart_and_wait`
already split it that way.

**`systemctl restart` returns long before the server is usable.** It spawns the
process and exits; the server then opens sockets and brings hardware up. Do not
follow a restart with a fixed `sleep` -- wait for the log line:

```
Server started on ws://0.0.0.0:8081
```

`restart_and_wait("x3_server", ready_pattern="Server started on ws")` does this.

Ask before restarting `x3_server` if the robot is physically powered on and
someone may be driving it.

## Reading logs

The journal is chatty -- per-frame estimator and FPS lines drown everything.
Filter both ways:

```bash
journalctl -u x3_server --since "-15min" --no-pager \
  | grep -iE 'error|traceback|exception' \
  | grep -viE 'Estimator\]|fps'
```

`journal_grep` takes `since`, `pattern`, and `exclude` directly.

## Serial devices

Code opens stable udev symlinks, not raw tty nodes:

| Symlink | Device | Hardware |
| --- | --- | --- |
| `/dev/openrb150` | `ttyACM0` | OpenRB-150 -> XL430-W250-T lidar tilt servo |
| `/dev/rosmaster` | `ttyCH341USB0` | ROSMaster base controller |
| `/dev/lx16a` | *(absent)* | Old LX-16A tilt servo, replaced by the Dynamixel |

`lsusb` showing the board is not enough -- the failure mode here is udev not
publishing the symlink, so the raw node exists and the code still cannot open
it. `check_robot_devices` checks the symlinks, the raw nodes, and the installed
rules together.

The XL430 runs at **1 Mbps**, not the 57600 factory default; it was written
into EEPROM on 2026-08-29. Its zero is **2048 counts**.

## Laptop and robot drift apart

The two copies of `src/` routinely diverge -- this has cost real debugging time.
Before changing robot code, check:

```
diff_code("/home/kamren/x3_ws/src/", "/home/jetson/x3_ws/src/")
```

It compares by checksum, ignores build artifacts, and says which side is ahead.
`show_diff="path/to/file.py"` gives the unified diff of one file.

`sync_code` defaults to `dry_run=True` -- read the plan, then call again with
`dry_run=False`. Do not pass `delete=True` unless you mean it; the robot has
files (maps, logs, model weights, fixtures) that exist nowhere else.

## Benchmarking

Kill orphaned nodes between runs or they keep publishing and poison the next
measurement: `cleanup_strays("bench")`, `dry_run=False` once you have read the
list.

`server_x3.py` is largely single-threaded; three of the Jetson's cores sit idle
while one saturates. Expect that when reading load numbers.
