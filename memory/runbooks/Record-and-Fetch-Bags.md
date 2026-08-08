---
title: Record and Fetch Bags
description: Runbook — record a rosbag on the robot, move it to the laptop, inspect it
tags: [runbook, data]
---

# Runbook: Record and Fetch Bags

Back to [[Home]]. Background: [[project_bag_transfer]], [[project_rosbag_corpus]].

## Record (on the Jetson)

```bash
chmod +x record_bag.sh
./record_bag.sh [OUTPUT_DIR]        # default ~/bags/domain_adapt/
```

Written for domain-adaptation collection: the robot is teleoperated while people walk around it.

To capture `/oak/*` topics you need the `--oak-ros-publish` drop-in — they are not published
by default. Watch for the env-var-prefix and `rclpy` `array.array` gotchas.
→ [[project_oak_rosbag_recording]]

## Move it to the laptop

The laptop has **no sshd**, so the transfer has to be initiated from the side that can reach
the other. Push from the Jetson:

```bash
scp -r ~/bags/domain_adapt/ <user>@<laptop-ip>:~/EE_244_Final_Project/bags/
```

`record_bag.sh` prints this line for you at the end and can push via `ssh -A` agent
forwarding. There is also a `fetch_bag.sh` for pulling — note it is currently **uncommitted**
in the working copy, so it won't be on the robot or in a fresh clone.

> [!warning] Non-interactive ssh has no ROS
> Any `ssh host 'command'` runs without the ROS environment. Source `/opt/ros/humble/setup.bash`
> (and the workspace overlay) explicitly inside the remote command or it will fail confusingly.

## Inspect

```bash
python3 src/bag_viewer.py --list
python3 src/bag_viewer.py --info <bag>
python3 src/bag_viewer.py <bag>          # GUI scrubber
```

It reads the bag sqlite directly — no DDS, no running ROS needed. → [[project_bag_viewer]]

## Before you trust a bag

The corpus lives in `EE_244_Final_Project/bags` and **42 bags are empty**. Check with
`--info` first. Truncated `.zstd` files can sometimes be recovered with the sqlite zero-pad
trick documented in [[project_rosbag_corpus]].

## Related

- [[Data-and-Bags]], [[Perception]]
