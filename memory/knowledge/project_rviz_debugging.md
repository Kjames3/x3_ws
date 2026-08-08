---
name: project_rviz_debugging
description: Ongoing investigation into why RViz on the laptop cannot receive ROS2 topic data from the Jetson despite discovery working
metadata: 
  node_type: memory
  type: project
  originSessionId: 8193f61d-1293-471f-b017-7663d5cffbc7
  modified: 2026-08-08T05:35:00.950Z
---

# RViz cross-machine data flow debugging

**RESOLVED 2026-06-20.** Root cause confirmed (discovery-server EDP forwarding broken) and
fixed by ABANDONING the discovery server for **default Simple discovery + unicast initial
peers**. UDP unicast works both ways across the school subnets (multicast does not). Fix:
laptop `scripts/laptop_viz.sh` generates a FastDDS profile with unicast peers at the Jetson's
metatraffic ports (must set BOTH `FASTRTPS_`/`FASTDDS_DEFAULT_PROFILES_FILE` — Humble's
Fast-DDS 2.6 only reads the legacy `FASTRTPS_` name); robot switched off the discovery server
via `scripts/jetson_simple_discovery.sh` drop-ins + `X3_SIMPLE_DISCOVERY=1` in `server_x3.py`.
RViz now receives /tf, /scan, /map. Everything below is historical.

## (historical) status as of 2026-05-22

## What works
- FastDDS discovery server on Jetson (10.13.199.34:11811, TCP+UDP)
- Laptop daemon with ROS_SUPER_CLIENT=TRUE sees all 27+ Jetson topics after `ros2 daemon stop && ros2 daemon start && sleep 5`
- UDP from Jetson→laptop on port 17911 confirmed (nc test)
- Hairpin routing on Jetson: Jetson can connect to own external IP 10.13.199.34:11811
- Camera opens correctly: /dev/video1 (sysfs fix deployed)

## Root cause identified
FastDDS 2.6.11 Discovery Server **EDP forwarding between regular clients is broken** in this setup.

Key observations:
- Super client (ROS_SUPER_CLIENT=TRUE): laptop sees all topics but subscriptions are "transparent" — publisher never learns subscriber exists, so data never flows
- Regular client: laptop's subscriber EDP is NOT forwarded by discovery server to Jetson publishers — Jetson always sees 0 subscribers for /scan even while laptop is actively subscribed
- Tested: TCP discovery server (TCPv4:), UDP discovery server (default format), with/without LARGE_DATA, with/without daemon, with/without --no-daemon — all result in 0 subscribers on Jetson

## Things tried and ruled out
- Firewall: UFW inactive on both machines
- Network: UDP unicast bidirectional (confirmed)
- FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA: removed, didn't help
- FASTDDS_DEFAULT_PROFILES_FILE XML: breaks cross-machine node discovery (reverted)
- ros2 topic echo --no-daemon: can't see topic type without super client, so fails
- CycloneDDS: not installed on either machine

## Current state of service files
- x3_server.service: ROS_DOMAIN_ID=42, ROS_DISCOVERY_SERVER=TCPv4:[127.0.0.1]:11811, NO FASTDDS_BUILTIN_TRANSPORTS
- orbbec_depth.service: same, NO FASTDDS_BUILTIN_TRANSPORTS
- Both deployed to Jetson and running

## Next approaches to try
1. **Enable FastDDS debug logging** to see if laptop EDP actually reaches the discovery server
2. **Reconfigure discovery server** — try running it as a "super server" or with different flags
3. **ros2_bridge relay** — run a bridge node on Jetson that republishes over a TCP socket
4. **X11 forwarding** — run RViz on Jetson, display on laptop via `ssh -X`
5. **Zenoh bridge** — zenoh-bridge-ros2dds over TCP (needs installation)
6. **SSH tunnel + loopback** — forward specific RTPS ports via SSH

## Why:
Needed for EE244 final project demo — laptop needs to visualize robot sensor data in RViz.
**How to apply:** when continuing this debugging, start from "Next approaches to try" above.
