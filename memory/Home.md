---
title: Home
description: Entry point for the X3 project memory vault
tags: [moc]
---

# X3 Project Memory

Vault root is the repository root, so every markdown file in `x3_ws` is a note here —
including [[CLAUDE]], [the root README](../README.md), the monthly idea logs, and the plan documents.
This `memory/` folder is the **organizing layer** on top of them.

> [!tip] Start here
> Lost? Open the [[#Maps of content]] below, or hit `Ctrl+O` and type what you remember.
> Search (`Ctrl+Shift+F`) is the real retrieval engine — see [[README-vault]].

## Maps of content

| Map | Covers |
|---|---|
| [[Hardware]] | Robot, Jetson, lidar, cameras, wiring, mounts |
| [[Software-Architecture]] | Server, ROS2 graph, packages, control flow |
| [[Navigation-and-SLAM]] | Odometry, EKF, SLAM Toolbox, Nav2, exploration |
| [[Perception]] | OAK-D, Astra, YOLO, TensorRT, velocity estimation |
| [[Data-and-Bags]] | Rosbag corpus, recording, transfer, viewing |
| [[Ideas-and-Planning]] | Idea logs, ROI analyses, plan docs, progress |

## Runbooks

- [[Build-and-Deploy]] — build the workspace, deploy to the Jetson, restart the service
- [[Bringup]] — bring the robot up and start the GUI
- [[Record-and-Fetch-Bags]] — record on the robot, pull to the laptop, inspect
- [[Troubleshooting-DDS]] — cross-machine ROS2 discovery problems

## Knowledge base

Atomic, durable facts — one per note, in `memory/knowledge/`.

- [[project_robot_deploy]] — SSH, systemd, git divergence, FastDDS quirks
- [[project_rviz_debugging]] — cross-machine RViz: topics visible, data never flows
- [[project_lidar_mount]] — lidar too low, sees chassis; CBF filter workaround
- [[project_oakd_lite]] — OAK-D Lite viewer config, USB3 vs USB2 fallback
- [[project_trt_engine]] — working FP16 yolo26n engine + detector fixes
- [[project_oak_rosbag_recording]] — publishing `/oak/*` into bags
- [[project_rosbag_corpus]] — where bags live, which are empty, repair trick
- [[project_bag_viewer]] — `src/bag_viewer.py` scrubber
- [[project_bag_transfer]] — push/pull bags between robot and laptop
- [[project_jetson_cpu_profile]] — py-spy profile, no HW encoder on Orin Nano

## Session log

Dated entries in `memory/log/`. Newest first.

- [[2026-08-07]] — vault created

## Conventions

Read [[README-vault]] before adding notes. Short version: one fact per note,
link generously, date everything, and never paste anything the code already says.
