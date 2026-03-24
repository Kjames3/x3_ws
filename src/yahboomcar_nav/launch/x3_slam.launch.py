"""
x3_slam.launch.py
-----------------
Full SLAM bringup for the Yahboom ROSMASTER X3 with YDLidar (TOF, 512kbaud).

Starts:
  - robot_state_publisher      (URDF → TF static frames)
  - joint_state_publisher
  - Mcnamu_driver_X3           (cmd_vel → Rosmaster hardware, publishes vel_raw / IMU)
  - base_node_X3               (vel_raw → /odom_raw, dead-reckoning odometry)
  - imu_filter_madgwick_node   (imu/data_raw → imu/data)
  - ekf_filter_node            (odom_raw + imu/data → /odom, odom→base_footprint TF)
  - ydlidar_ros2_driver_node   (/scan laser scan)
  - async_slam_toolbox_node    (/scan + odom TF → /map)

Run:
  ros2 launch yahboomcar_nav x3_slam.launch.py

Then drive the robot via the web GUI (python3 src/server_x3.py --ros2) or a joystick.
Save the finished map:
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '{name: {data: "/home/kamren/x3_ws/maps/mymap"}}'
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────
    desc_share  = get_package_share_directory('yahboomcar_description')
    bringup_share = get_package_share_directory('yahboomcar_bringup')
    nav_share   = get_package_share_directory('yahboomcar_nav')

    # ── Paths ─────────────────────────────────────────────────────────
    urdf_path           = os.path.join(desc_share,  'urdf', 'yahboomcar_X3.urdf')
    imu_filter_cfg      = os.path.join(bringup_share, 'param', 'imu_filter_param.yaml')
    ekf_cfg             = os.path.join(nav_share, 'params', 'ekf_x3.yaml')
    ydlidar_cfg         = os.path.join(nav_share, 'params', 'ydlidar_x3.yaml')
    slam_cfg            = os.path.join(nav_share, 'params', 'slam_toolbox_params.yaml')

    # ── Launch args ───────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='false', choices=['true', 'false'],
        description='Launch joint_state_publisher_gui instead of headless')

    # ── Robot description ─────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # ── Hardware driver ───────────────────────────────────────────────
    # Subscribes /cmd_vel → Rosmaster serial; publishes /vel_raw, /imu/data_raw
    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        output='screen',
    )

    # ── Velocity-based odometry ───────────────────────────────────────
    # /vel_raw → /odom_raw (dead-reckoning)
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        output='screen',
        parameters=[{
            'pub_odom_tf': False,       # EKF publishes odom→base_footprint TF
            'linear_scale_x': 1.0,
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }]
    )

    # ── IMU filter ────────────────────────────────────────────────────
    # /imu/data_raw → /imu/data  (Madgwick orientation filter)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[imu_filter_cfg],
        remappings=[('/imu/data_raw', '/imu/data_raw')]
    )

    # ── EKF localisation ─────────────────────────────────────────────
    # /odom_raw + /imu/data → /odom  +  odom→base_footprint TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_cfg],
    )

    # ── YDLidar ───────────────────────────────────────────────────────
    # Publishes /scan (sensor_msgs/LaserScan) in frame laser_link
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[ydlidar_cfg],
    )

    # ── SLAM Toolbox ─────────────────────────────────────────────────
    # /scan + odom TF → /map  +  map→odom TF
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg],
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        driver_node,
        base_node,
        imu_filter_node,
        ekf_node,
        ydlidar_node,
        slam_toolbox_node,
    ])
