"""
x3_remote_viz.launch.py
-----------------------
RViz2-only launch for visualising the real robot from a laptop.
Does NOT start any hardware drivers — connect to the Jetson over ROS_DOMAIN_ID.

Arguments:
  rvizconfig  <path>  Absolute path to a .rviz file.
                      Default: x3_full_viz.rviz — robot model + lidar + RGB camera
                               + depth image + DepthCloud (3D point cloud) + SLAM map.
                      Pass map.rviz for a minimal SLAM/lidar-only view.
                      Pass nav.rviz for the Nav2 goal-clicking view.

Usage (run on laptop while jetson_bringup.sh is running on the robot):

  # Convenience wrapper (sets ROS_DOMAIN_ID automatically):
  bash scripts/laptop_viz.sh [DOMAIN_ID]       # default domain 42

  # Full view (default) — robot + lidar + camera + depth cloud + map
  export ROS_DOMAIN_ID=42
  ros2 launch yahboomcar_nav x3_remote_viz.launch.py

  # Lidar + SLAM map only (lightweight)
  export ROS_DOMAIN_ID=42
  ros2 launch yahboomcar_nav x3_remote_viz.launch.py \\
      rvizconfig:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/map.rviz

  # Nav2 goal view — costmaps, planned path, click to set goals
  export ROS_DOMAIN_ID=42
  ros2 launch yahboomcar_nav x3_remote_viz.launch.py \\
      rvizconfig:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/nav.rviz

Laptop prerequisites:
  - ROS2 Humble installed (/opt/ros/humble/)
  - This workspace built: colcon build --packages-select yahboomcar_nav yahboomcar_description yahboomcar_msgs
  - Same LAN as Jetson with matching ROS_DOMAIN_ID

Topics consumed from Jetson (no local drivers needed):
  /robot_description          — URDF for RobotModel display
  /scan                       — YDLidar LaserScan (~8 Hz)
  /odom                       — EKF-fused odometry
  /map                        — SLAM OccupancyGrid (when SLAM is running)
  /camera/depth/image_raw     — Orbbec Astra depth image (16UC1, millimetres)
  /camera/depth/camera_info   — Depth camera intrinsics (required by DepthCloud)
  /tf + /tf_static            — Full TF tree (odom→base_footprint→sensors)

NOTE: The Orbbec camera driver is NOT started by x3_bringup.launch.py.
Start it separately on the Jetson before recording or visualising depth data.
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav_pkg = get_package_share_directory('yahboomcar_nav')

    # ── Launch argument ───────────────────────────────────────────────────
    # Expose the RViz2 config as a direct path so the user can easily swap
    # between map.rviz (SLAM) and nav.rviz (Nav2) or a custom file.
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(nav_pkg, 'rviz', 'x3_full_viz.rviz'),
        description=(
            'Absolute path to RViz2 config file. '
            'x3_full_viz.rviz = full view (robot + lidar + camera + depth + map). '
            'map.rviz = SLAM-only view (/scan + /map). '
            'nav.rviz = Nav2 goal tool (costmaps + path).'
        ),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    # Subscribes to topics broadcast by the Jetson over the shared domain.
    # map.rviz displays:  RobotModel, TF, LaserScan (/scan), Map (/map)
    # nav.rviz displays:  all of the above + global/local costmaps,
    #                     Nav2 planned path, and the "2D Goal Pose" tool
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
    ])
