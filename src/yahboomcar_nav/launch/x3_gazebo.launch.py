"""
x3_gazebo.launch.py
Launches Ignition Fortress (gz-sim 6) with the Yahboom X3 mecanum robot.

Nodes started:
  1. gz_sim server + GUI       (ros_gz_sim)
  2. robot_state_publisher     (broadcasts TF from URDF)
  3. joint_state_publisher     (publishes wheel joint states)
  4. ros_gz_sim create         (spawns robot into Ignition world, delayed 5 s)
  5. ros_gz_bridge             (bridges Ignition Transport ↔ ROS 2 topics)

Topics published after bridge:
  /scan        — sensor_msgs/LaserScan  (from /lidar via ros_gz_bridge)
  /odom        — nav_msgs/Odometry      (from model odometry via ros_gz_bridge)
  /imu/data    — sensor_msgs/Imu        (from /imu_raw via ros_gz_bridge)

Topics subscribed:
  /cmd_vel     — geometry_msgs/Twist    (bridged to VelocityControl plugin)

Run standalone:
  ros2 launch yahboomcar_nav x3_gazebo.launch.py

Run with server in sim mode (server auto-calls this via subprocess):
  python3 server_x3.py --sim
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg   = get_package_share_directory('yahboomcar_description')
    nav_pkg    = get_package_share_directory('yahboomcar_nav')
    ros_gz_pkg = get_package_share_directory('ros_gz_sim')

    world_file  = os.path.join(nav_pkg, 'worlds', 'x3_world.world')
    robot_xacro = os.path.join(desc_pkg, 'urdf', 'yahboomcar_X3_gazebo.urdf.xacro')

    robot_description = ParameterValue(
        Command(['/opt/ros/humble/bin/xacro ', robot_xacro]),
        value_type=str
    )

    # ── 0. Set mesh resource path for Ignition Fortress ───────────────────
    # Ignition converts package:// URIs to model:// and searches for
    # 'yahboomcar_description/meshes/...' relative to GZ_SIM_RESOURCE_PATH.
    # desc_pkg is  .../share/yahboomcar_description  — we need its parent
    # (.../share) so that 'yahboomcar_description' is a subdirectory of the path.
    gz_resource_dir = os.path.dirname(desc_pkg)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path_value = f'{gz_resource_dir}:{existing}' if existing else gz_resource_dir

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path_value,
    )
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path_value,
    )

    # ── 1. Ignition Fortress (server + GUI client) ─────────────────────────
    # -r flag starts the simulation running immediately (not paused)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
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

    # ── 4. Spawn robot into Ignition world (delayed 5 s) ──────────────────
    # Fortress takes longer to initialise than Classic — 5 s is safer than 3 s.
    # ros_gz_sim 'create' reads robot_description from the ROS parameter server.
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_x3',
                output='screen',
                arguments=[
                    '-name',  'yahboomcar_x3',
                    '-topic', 'robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '0.1',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                parameters=[{'use_sim_time': True}],
            )
        ],
    )

    # ── 5. ROS ↔ Ignition topic bridge ────────────────────────────────────
    # Format: "IGN_TOPIC@ROS_MSG_TYPE[ign_msg_type"  ([ = Ignition→ROS)
    #         "IGN_TOPIC@ROS_MSG_TYPE]ign_msg_type"  (] = ROS→Ignition)
    #
    # VelocityControl in Fortress listens on /model/<name>/cmd_vel by default.
    # If the robot doesn't respond to /cmd_vel, check 'ign topic -l' and update
    # the first entry to '/model/yahboomcar_x3/cmd_vel@...' with a remapping.
    bridge_params = [
        # cmd_vel: ROS → Ignition (drives VelocityControl plugin)
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        # LiDAR: Ignition → ROS
        '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        # IMU: Ignition → ROS  (remapped /imu_raw → /imu/data below)
        '/imu_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        # Odometry: Ignition → ROS  (remapped to /odom below)
        '/model/yahboomcar_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=bridge_params,
        remappings=[
            ('/imu_raw', '/imu/data'),
            ('/model/yahboomcar_x3/odometry', '/odom'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        set_gz_resource,
        set_ign_resource,
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
