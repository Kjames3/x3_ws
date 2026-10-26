# EE 244 Final Project - Predictive Local Planning via Onboard Velocity Estimation

This is the final project for the EE 244 Computational Learning Course at UCR (Spring 2026), created by Kamren James.

## Project Overview

Mobile robots navigating among people must avoid dynamic obstacles whose future positions depend on their current velocity. Standard local planners, such as the Dynamic Window Approach (DWA), treat obstacles as static within each planning cycle or assume constant velocity heuristically, producing late and reactive maneuvers. 

This project addresses the problem: **How can a mobile robot accurately estimate the velocity of nearby dynamic obstacles directly from onboard RGB-D and LiDAR sensor observations and use those estimates to plan collision-free paths predictively rather than reactively?**

## Approach

The current direction involves a lightweight MLP regression network trained on sequences of depth-image centroid displacements and LiDAR point-cloud features to output per-obstacle 2D velocity vectors. These estimates drive a modified DWA local planner running in ROS2.

The hardware platform is a Yahboom X3 holonomic-drive robot equipped with:
- Jetson Orin Nano Super 8GB
- Orbbec Astra Pro SC RGB-D camera
- YDLidar 4ROS TOF LiDAR

## Datasets

The project leverages several datasets:
- **COCO**: Pre-training backbone for person detection.
- **JRDB**: Fine-tuning for detection.
- **THÖR-MAGNI**: Primary dataset for training and validating the velocity regression model, providing 100 Hz motion-capture velocity ground truth alongside synchronized RGB-D and LiDAR streams.
- **Custom Domain-Adaptation Dataset**: Collected with the Orbbec Astra Pro SC in the deployment environment to bridge the sensor domain gap.

## Results to Demonstrate

1. Quantitative velocity estimation accuracy on held-out THÖR-MAGNI sequences against a Kalman-filter baseline.
2. Navigation metrics (success rate, minimum clearance distance, time-to-goal) evaluated on the physical Yahboom X3.
3. A live classroom demo comparing reactive vs. predictive A/B with DWA velocity estimation disabled and enabled.
