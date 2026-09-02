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
    lidar3d_cfg         = os.path.join(nav_share, 'params', 'lidar_3d_processor.yaml')
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

    # The headless joint_state_publisher is GONE, deliberately.  server_x3.py's
    # tilt sampler now publishes /joint_states directly (see ROS2Bridge.
    # full_joint_pub), carrying the four wheel joints at 0.0 exactly as this
    # node did plus the real lidar_tilt_joint.
    #
    # The `source_list` relay it replaces was measured on the robot 2026-08-29
    # and is unfixable for this job: it pins TF at 10.0 Hz however fast it is
    # fed (10/25/50/100 Hz in -> 8.9/10.0/10.0/10.0 Hz out), reaches only
    # 15.7-18 Hz even at `rate:=100` while burning 28% of a core, and RE-STAMPS
    # ~34% of what it emits with its own timer's time instead of the
    # measurement time.  Per-ray deskew interpolates TF between the stamps
    # either side of a scan, so both behaviours corrupt it.

    # gui:=true is a URDF-inspection mode ONLY.  joint_state_publisher_gui
    # publishes every joint including lidar_tilt_joint, so it fights
    # server_x3.py's tilt sampler for the real angle -- do not use it while the
    # mount is live.
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
        parameters=[{
            'wheel_separation_factor': 0.165,
            'gain_fl': 1.00,
            'gain_fr': 0.95,
            'gain_rl': 1.00,
            'gain_rr': 0.95,
            # The ICM-42688-P (icm42688_node, 200 Hz on i2c-7) now owns
            # imu/data_raw. Two publishers on one topic interleave silently.
            # This gates ONLY imu/data_raw + imu/mag; the MPU9250 gyro remains
            # the angular-velocity source for vel_raw, because the firmware's
            # own value is wrong (M2/M3 encoder cables swapped).
            'publish_imu': False,
            # Take vel_raw's angular.z from the ICM too, so the MPU9250 leaves
            # the heading loop entirely. Without this the EKF still anchors
            # absolute yaw on the old sensor's bias via odom0.
            'use_external_imu_yaw': True,
        }]
    )

    # ── Velocity-based odometry ───────────────────────────────────────
    # /vel_raw → /odom_raw (dead-reckoning)
    base_node = Node(
        package='yahboomcar_base_node',
        executable='base_node_X3',
        output='screen',
        parameters=[{
            'pub_odom_tf': False,       # EKF publishes odom→base_footprint TF
            'linear_scale_x': -1.0,    # encoder convention: forward = negative ticks
            'linear_scale_y': 1.0,
            'angular_scale': 1.0,
        }]
    )

    # ── IMU: ICM-42688-P on i2c-7 @ 0x68 ─────────────────────────────
    # Replaces the MPU9250, which reached ROS at 10 Hz through the Rosmaster's
    # 115200-baud serial bridge. 200 Hz, Jetson-side timestamps, measured gyro
    # bias subtracted from config/icm42688_calibration.json.
    icm_imu_node = Node(
        package='yahboomcar_bringup',
        executable='icm42688_node',
        name='icm42688_node',
        output='screen',
        parameters=[{
            'i2c_bus': 7,          # pins 3/5, shared with the INA226 at 0x40
            'i2c_addr': 0x68,      # AD0 tied to GND
            'topic': 'imu/data_raw',
            'frame_id': 'icm_imu_link',
            'rate': 200.0,
        }],
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
        remappings=[('/scan', '/scan_raw')]
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
        joint_state_publisher_gui,
        driver_node,
        base_node,
        icm_imu_node,
        imu_filter_node,
        ekf_node,
        ydlidar_node,
        Node(
            package='yahboomcar_bringup',
            executable='lidar_3d_processor_node',
            name='lidar_3d_processor_node',
            output='screen',
            # Was launched with NO parameter block at all, which pinned every
            # knob (tilt timeout, range mask, publish_cloud_when_level,
            # require_settled) at its code default and made the node
            # untunable from launch.
            parameters=[lidar3d_cfg],
        ),
        slam_toolbox_node,
    ])
