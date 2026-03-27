"""
x3_slam_sim.launch.py
---------------------
SLAM Toolbox bringup for the Yahboom X3 in Gazebo Ignition Fortress simulation.

This is intentionally minimal — Gazebo already provides all sensor topics via
ros_gz_bridge (launched by x3_gazebo.launch.py), so no hardware drivers are needed:
  /scan              ← ros_gz_bridge (GPU lidar → LaserScan)
  /odom              ← ros_gz_bridge (model odometry)
  /imu/data          ← ros_gz_bridge (Ignition IMU)
  odom→base_footprint TF ← ros_gz_bridge or robot_state_publisher

All this launch does is start async_slam_toolbox_node with use_sim_time=true,
which reads /scan and the odom→base_footprint TF to build and publish /map.

Run alongside the Gazebo launch:
  Terminal 1: ros2 launch yahboomcar_nav x3_gazebo.launch.py
  Terminal 2: ros2 launch yahboomcar_nav x3_slam_sim.launch.py

Or, if using the web server in --sim mode, just click "Start SLAM" in the GUI —
server_x3.py manages this launch as a subprocess automatically.

Save the finished map via the GUI "Save Map" button, or from the CLI:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    '{name: {data: "/home/kamren/x3_ws/src/yahboomcar_nav/maps/mymap"}}'
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('yahboomcar_nav')
    slam_cfg  = os.path.join(nav_share, 'params', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock (should always be true for this launch)')

    # async_slam_toolbox_node subscribes to /scan and the odom TF chain,
    # then publishes /map and the map→odom TF correction.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_cfg,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node,
    ])
