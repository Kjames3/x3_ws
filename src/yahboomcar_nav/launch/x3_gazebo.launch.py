"""
x3_gazebo.launch.py
Launches Gazebo Classic with the Yahboom X3 mecanum robot.

Nodes started:
  1. Gazebo server + client  (gazebo_ros)
  2. robot_state_publisher   (broadcasts TF from URDF)
  3. joint_state_publisher   (publishes wheel joint states)
  4. spawn_entity            (drops the robot into the world)

Topics published by Gazebo plugins (from yahboomcar_X3_gazebo.urdf.xacro):
  /scan        — sensor_msgs/LaserScan  (libgazebo_ros_ray_sensor)
  /odom        — nav_msgs/Odometry      (libgazebo_ros_planar_move)
  /imu/data    — sensor_msgs/Imu        (libgazebo_ros_imu_sensor)

Topics subscribed by Gazebo:
  /cmd_vel     — geometry_msgs/Twist    (libgazebo_ros_planar_move)

Run standalone:
  ros2 launch yahboomcar_nav x3_gazebo.launch.py

Run with server in sim mode (server auto-calls this via subprocess):
  python3 server_x3.py --sim
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg  = get_package_share_directory('yahboomcar_description')
    nav_pkg   = get_package_share_directory('yahboomcar_nav')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    world_file  = os.path.join(nav_pkg, 'worlds', 'x3_world.world')
    robot_xacro = os.path.join(desc_pkg, 'urdf', 'yahboomcar_X3_gazebo.urdf.xacro')

    robot_description = Command(['xacro ', robot_xacro])

    # ── 1. Gazebo (server + GUI client) ────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   world_file,
            'verbose': 'false',
            'pause':   'false',
        }.items(),
    )

    # ── 2. Robot State Publisher (URDF → TF) ───────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── 3. Joint State Publisher ───────────────────────────────────────────
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    # ── 4. Spawn robot (delayed 3 s so Gazebo is ready) ───────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_x3',
                output='screen',
                arguments=[
                    '-entity', 'yahboomcar_x3',
                    '-topic',  'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.1',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
            )
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
