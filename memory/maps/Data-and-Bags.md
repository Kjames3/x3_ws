---
title: Data and Bags
description: Map of content — rosbag corpus, recording, transfer, inspection
tags: [moc, data]
---

# Data and Bags

Back to [[Home]].

## The corpus

Bags live in `EE_244_Final_Project/bags`. **42 of them are empty** — check before you spend
time on one. Truncated `.zstd` files and the sqlite zero-pad repair trick are documented in
[[project_rosbag_corpus]].

## Recording

`record_bag.sh` on the robot. Getting `/oak/*` topics into a bag needs the
`--oak-ros-publish` drop-in; watch for the env-var-prefix and `rclpy` `array.array` gotchas.
→ [[project_oak_rosbag_recording]]

**Do a short trial first.** Record a brief timed bag and check it at the payload level —
message counts per topic, not just that files exist — before committing to a long capture
session. Too many sessions have produced empty bags. → [[feedback_verify_before_long_runs]]

## Transfer

Push from the robot with `record_bag.sh` (uses `ssh -A` agent forwarding); pull with
`fetch_bag.sh`. The laptop has no sshd, so the direction matters. Non-interactive ssh
sessions don't have ROS on the path — source it explicitly in any remote command.
→ [[project_bag_transfer]]

## Inspection

`src/bag_viewer.py` — GUI scrubber plus `--list` / `--info`. It reads the bag sqlite directly,
so it needs no DDS and works on a machine with no ROS running. → [[project_bag_viewer]]

## Related

- [[Perception]], [[Software-Architecture]], [[Troubleshooting-DDS]]
