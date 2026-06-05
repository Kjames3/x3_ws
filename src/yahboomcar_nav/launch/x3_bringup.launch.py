"""
x3_bringup.launch.py
--------------------
Hardware-only bringup for the Yahboom ROSMASTER X3.
Starts all physical drivers and state estimation WITHOUT SLAM Toolbox.

Starts:
  - robot_state_publisher      (URDF → TF static frames + /robot_description)
  - joint_state_publisher
  - Mcnamu_driver_X3           (cmd_vel → Rosmaster hardware, publishes vel_raw / imu)
  - base_node_X3               (vel_raw → /odom_raw, dead-reckoning odometry)
  - imu_filter_madgwick_node   (imu/data_raw → imu/data)
  - ekf_filter_node            (odom_raw + imu/data → /odom, odom→base_footprint TF)
  - ydlidar_ros2_driver_node   (/scan laser scan)
  - orbbec_depth_node          (/camera/depth/image_raw + /camera/depth/camera_info)

NOTE: orbbec_depth_node opens the Orbbec Astra via OpenNI2 directly.
      Do NOT run this alongside server_x3.py — both cannot hold the device at once.

Verify all sensors after launch:
  ros2 topic hz /scan                    # ~8 Hz
  ros2 topic hz /camera/depth/image_raw  # ~30 Hz
  ros2 topic hz /odom                    # ~50 Hz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────
    desc_share    = get_package_share_directory('yahboomcar_description')
    bringup_share = get_package_share_directory('yahboomcar_bringup')
    nav_share     = get_package_share_directory('yahboomcar_nav')

    # ── Paths ─────────────────────────────────────────────────────────
    urdf_path      = os.path.join(desc_share,    'urdf',  'yahboomcar_X3.urdf')
    imu_filter_cfg = os.path.join(bringup_share, 'param', 'imu_filter_param.yaml')
    ekf_cfg        = os.path.join(nav_share,     'params', 'ekf_x3.yaml')
    ydlidar_cfg    = os.path.join(nav_share,     'params', 'ydlidar_x3.yaml')

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
    serial_port = '/dev/ttyCH341USB0'
    if os.path.exists('/dev/ttyCH341USB1'):
        serial_port = '/dev/ttyCH341USB1'
    elif os.path.exists('/dev/ttyCH341USB0'):
        serial_port = '/dev/ttyCH341USB0'

    driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            # (half_wheelbase + half_track) from URDF: (0.08 + 0.0845) = 0.1645 m
            'wheel_separation_factor': 0.165,
            # Per-wheel gain: reduces faster motor side to correct drift/circle.
            # Robot drifts right → right is stronger → gain_fr/rr < 1.0.
            # Tune with: ros2 param set /driver_node gain_fr 0.93
            'gain_fl': 1.00,
            'gain_fr': 0.95,
            'gain_rl': 1.00,
            'gain_rr': 0.95,
        }]
    )

    # ── Velocity-based odometry ───────────────────────────────────────
    # /vel_raw → /odom_raw (dead-reckoning)
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        output='screen',
        parameters=[{
            'pub_odom_tf': False,
            'linear_scale_x': -1.0,   # encoder convention: forward = negative ticks
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
        remappings=[('/odometry/filtered', '/odom')],
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

    # ── Orbbec depth camera ───────────────────────────────────────────
    # Publishes /camera/depth/image_raw (16UC1, mm) + /camera/depth/camera_info
    # Required for record_bag.sh and laptop RViz visualization.
    _camera_script = os.path.join(
        os.path.dirname(__file__), '..', 'orbbec_depth_node.py'
    )
    camera_node = ExecuteProcess(
        cmd=['python3', _camera_script],
        output='screen',
        name='orbbec_depth_node',
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
        # camera_node,
    ])
