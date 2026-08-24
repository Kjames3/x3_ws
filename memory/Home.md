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

This list is generated — run `python3 scripts/import_auto_memory.py` to refresh it
along with the notes themselves. Don't hand-edit between the markers.

<!-- knowledge-index:start -->
- [[feedback_verify_before_long_runs]] — Kamren validates a recording pipeline with a short trial bag and a payload-level check before committing to a long capture session
- [[project_august_ideas]] — august_improvement_ideas.md / august_roi_analysis.md — dual authorship by a scheduled cloud routine and local sessions, and the A-NN numbering hazard
- [[project_bag_transfer]] — Bag auto-transfer robot↔laptop — push in record_bag.sh, pull via fetch_bag.sh; laptop sshd + non-interactive ROS gotchas
- [[project_bag_viewer]] — src/bag_viewer.py — GUI/CLI for browsing and scrubbing the robot's rosbags
- [[project_dream_skill]] — The /dream memory-curation skill — where it is installed from, where its report and backups live, how apply is guarded, and why overnight runs are not happening
- [[project_jetson_cpu_profile]] — X3 server CPU profile on the Jetson Orin Nano — top consumers, the psutil fix, and where CPU actually goes
- [[project_lidar_mount]] — Lidar is mounted low and sees the chassis; temporary CBF filter workarounds are in place pending a bracket to raise it
- [[project_oak_rosbag_recording]] — How to record OAK-D Lite data to rosbags — requires --oak-ros-publish systemd drop-in; env-var and rclpy gotchas; /oak/detections is fabricated
- [[project_oakd_lite]] — OAK-D Lite on the X3 — how it is driven, the USB-C cable trap, measured hardware ceilings, and the custom yolo26 host-decode path
- [[project_obsidian_vault]] — x3_ws repo root doubles as an Obsidian vault with a memory/ layer — why nothing was moved, and why RAG was rejected at this scale
- [[project_robot_deploy]] — How to reach/inspect the live X3 robot (jetson) and how its deploy diverges from local git
- [[project_rosbag_corpus]] — State of the EE_244_Final_Project rosbag corpus + how to read/repair these zstd bags
- [[project_rosmaster_lib]] — Rosmaster_Lib.py hardware-measured facts — 2ms gap is load-bearing, MPU9250 not ICM, mag scale wrong, baud can't be raised
- [[project_rviz_debugging]] — Ongoing investigation into why RViz on the laptop cannot receive ROS2 topic data from the Jetson despite discovery working
- [[project_trt_engine]] — Working TensorRT engine for yolo26n on the robot — how it was built, the trt_detector.py fixes, and when it actually runs
<!-- knowledge-index:end -->

## Session log

Dated entries in `memory/log/`. Newest first.

- [[2026-08-08]] — scripted auto-memory import; vault refreshed
- [[2026-08-07]] — vault created

## Conventions

Read [[README-vault]] before adding notes. Short version: one fact per note,
link generously, date everything, and never paste anything the code already says.
