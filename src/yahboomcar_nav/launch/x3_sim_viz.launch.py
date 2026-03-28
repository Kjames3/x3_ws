"""
x3_sim_viz.launch.py
--------------------
Single-command launch for offline simulation on a laptop:
  - Ignition Fortress (Gazebo) with the X3 robot + ROS bridge
  - RViz2 with the appropriate config (SLAM view or Nav2 view)

Arguments:
  rviz         [true|false]  Open RViz2 alongside Gazebo (default: true)
  rvizconfig   <path>        Absolute path to a .rviz file.
                             Default: map.rviz (live /scan + /map view).
                             Pass nav.rviz for the Nav2 goal-clicking view.

Usage:
  # Gazebo + RViz2 with SLAM map view (default)
  ros2 launch yahboomcar_nav x3_sim_viz.launch.py

  # Gazebo + RViz2 with Nav2 goal view
  ros2 launch yahboomcar_nav x3_sim_viz.launch.py \\
      rvizconfig:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/nav.rviz

  # Gazebo only, no RViz2 window
  ros2 launch yahboomcar_nav x3_sim_viz.launch.py rviz:=false

Typical sim workflow:
  Terminal 1: ros2 launch yahboomcar_nav x3_sim_viz.launch.py
  Terminal 2: python3 src/server_x3.py --sim
  Browser:    open GUI, drive robot, click "Start SLAM" to build a map
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav_pkg = get_package_share_directory('yahboomcar_nav')

    # ── Launch arguments ──────────────────────────────────────────────────

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Set false to run Gazebo headless (no RViz2 window)',
    )

    # Expose the RViz2 config as a direct path argument so the user can
    # swap between map.rviz (SLAM) and nav.rviz (Nav2) or a custom file.
    rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(nav_pkg, 'rviz', 'map.rviz'),
        description=(
            'Absolute path to RViz2 config file. '
            'Use map.rviz for live SLAM view, nav.rviz for Nav2 goal tool.'
        ),
    )

    # ── Ignition Fortress + X3 robot + ROS↔Ignition topic bridge ──────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'x3_gazebo.launch.py')
        ),
    )

    # ── RViz2 — delayed 6 s so Gazebo and the bridge are ready ────────────
    # map.rviz  →  /scan laser, /map occupancy, TF frames, RobotModel
    # nav.rviz  →  /map, global/local costmaps, planned path, Nav2 goal tool
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rvizconfig')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')),
            ),
        ],
    )

    return LaunchDescription([
        rviz_arg,
        rviz_config_arg,
        gazebo,
        rviz_node,
    ])
