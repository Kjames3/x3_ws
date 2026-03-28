"""
x3_remote_viz.launch.py
-----------------------
RViz2-only launch for visualising the real robot from a laptop.
Does NOT start any hardware drivers — connect to the Jetson over ROS_DOMAIN_ID.

Arguments:
  rvizconfig  <path>  Absolute path to a .rviz file.
                      Default: map.rviz (live /scan + SLAM map view).
                      Pass nav.rviz for the Nav2 goal-clicking view.

Usage (run on laptop while jetson_bringup.sh is running on the robot):

  # Live SLAM map view (default) — see lidar scan + map being built
  export ROS_DOMAIN_ID=42
  ros2 launch yahboomcar_nav x3_remote_viz.launch.py

  # Nav2 goal view — see costmaps, planned path, click to set goals
  export ROS_DOMAIN_ID=42
  ros2 launch yahboomcar_nav x3_remote_viz.launch.py \\
      rvizconfig:=$(ros2 pkg prefix yahboomcar_nav)/share/yahboomcar_nav/rviz/nav.rviz

Map viewing workflow after saving a map in the GUI:
  Jetson:  ros2 launch yahboomcar_nav x3_nav2.launch.py \\
               map:=$HOME/x3_ws/src/yahboomcar_nav/maps/<map_name>.yaml
  Laptop:  export ROS_DOMAIN_ID=42
           ros2 launch yahboomcar_nav x3_remote_viz.launch.py \\
               rvizconfig:=.../rviz/nav.rviz
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
        default_value=os.path.join(nav_pkg, 'rviz', 'map.rviz'),
        description=(
            'Absolute path to RViz2 config file. '
            'map.rviz = live SLAM view (/scan + /map). '
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
