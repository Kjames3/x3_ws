---
title: Bringup
description: Runbook — start the robot stack and the web GUI
tags: [runbook]
---

# Runbook: Bringup

Back to [[Home]]. ROS2 hardware bridge mode is the **default**; `--sim` turns it off.

## A — Physical robot, full ROS2 stack

```bash
bash scripts/jetson_bringup.sh [DOMAIN_ID]     # default domain 42
python3 src/server_x3.py --domain-id 42
```

## B — Simulation on the laptop, server on the Jetson

```bash
bash scripts/laptop_sim.sh [DOMAIN_ID]         # on the laptop
python3 src/server_x3.py --domain-id 42        # on the Jetson
```

## C — Everything simulated on one machine

```bash
python3 src/server_x3.py --sim
```

## Auto-start

Service file `src/x3_server.service` (runs `--domain-id 42`). Override with
`/etc/systemd/system/x3_server.service.d/override.conf`. See [[Build-and-Deploy]] for the
restart caveats — they matter.

## GUI

Browser → HTTP `:8080` for the files, WebSocket `:8081` for control. The HTTP server is
**required**: opening `GUI.html` from the filesystem breaks the Web Worker that loads
`lidar-worker.js`.

## Inspect a running server without ROS

Connect to `ws://localhost:8081` on the robot and read `readout` messages. Useful fields:

- `velocity_estimates`
- `depth_image` — only after sending `{"type":"toggle_depth","enabled":true}`
- `detections` — only after `{"type":"toggle_detection","enabled":true}`; **defaults OFF and
  resets OFF on every restart**, which has cost debugging time more than once

## Other scripts

`scripts/`: `jetson_bringup.sh`, `laptop_sim.sh`, `laptop_viz.sh`, `map_classroom.sh`,
`jetson_simple_discovery.sh`, `open_ros2_firewall.sh`, `build_ros2.sh`, `install.sh`.

## Related

- [[Build-and-Deploy]], [[Troubleshooting-DDS]], [[Navigation-and-SLAM]]
