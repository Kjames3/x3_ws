---
title: Troubleshooting DDS
description: Runbook — cross-machine ROS2 discovery and "topics visible but no data"
tags: [runbook, ros2]
---

# Runbook: Troubleshooting DDS

Back to [[Home]]. This has been the single largest time sink on the project —
read this **before** debugging the planner, the driver, or RViz.

## Current configuration

As of 2026-07-25 the running system uses **plain default DDS multicast on
`ROS_DOMAIN_ID=42`** — *not* the FastDDS discovery server.

Verified by reading `/proc/<pid>/environ` of the live `x3_server` **and** `Mcnamu_driver_X3`:
both have only `ROS_DOMAIN_ID=42`. No `ROS_DISCOVERY_SERVER`, no
`FASTDDS_DEFAULT_PROFILES_FILE`, no `RMW_IMPLEMENTATION`.

The older discovery-server setup (`ROS_DISCOVERY_SERVER=127.0.0.1:11811`,
`FASTDDS_DEFAULT_PROFILES_FILE=.../config/fastdds_tcp_server.xml`) is **no longer in use**.
Ignore instructions that assume it.

## The central trap: discovery works, data doesn't

> [!warning] A zero-message reading is NOT proof of silence
> Graph introspection (`ros2 topic list`, `ros2 topic info -v`) is **reliable** — it correctly
> shows topics and publisher/subscriber counts.
>
> But an ad-hoc late-joining subscriber — `ros2 topic echo`, `ros2 topic hz`, or your own
> throwaway node — has been observed receiving **zero data on every topic** (`/scan`, `/odom`,
> `/cmd_vel`) while everything was in fact publishing normally. This is a data-transport quirk
> of late-joining participants, not evidence the publisher is dead.

So: use graph introspection to answer "does this topic exist and is something publishing to
it". Do **not** conclude "nothing is flowing" from an empty `echo`.

## Instead, verify through the server

Connect to `ws://localhost:8081` on the robot and read `readout` messages. That path doesn't
involve DDS at all, so it distinguishes "the robot isn't producing data" from "my subscriber
can't receive it". See [[Bringup]].

## Cross-machine RViz

The laptop can list topics while data never arrives — the same failure above, plus historical
FastDDS EDP forwarding problems. Details and next steps: [[project_rviz_debugging]].

Firewall helper: `scripts/open_ros2_firewall.sh`. Domain isolation is why everything uses 42.

## Checklist

1. Same `ROS_DOMAIN_ID` on both ends? (42)
2. No stale `ROS_DISCOVERY_SERVER` / `FASTDDS_DEFAULT_PROFILES_FILE` exported in your shell?
3. `ros2 topic info -v` shows the expected publisher count?
4. If yes but `echo` is empty — suspect the late-joiner quirk, not the publisher.
5. Confirm real data via the WebSocket readout before changing any code.
6. Non-interactive `ssh` has **no ROS on the path** — source it explicitly in remote commands.

## Related

- [[project_rviz_debugging]], [[project_robot_deploy]], [[Build-and-Deploy]], [[Bringup]]
