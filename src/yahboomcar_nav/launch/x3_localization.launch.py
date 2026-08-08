"""
x3_localization.launch.py
Localization ONLY — map_server + AMCL + lifecycle manager. No planner, no
controller, no behaviour tree.

Why separate from x3_nav2.launch.py:
  x3_nav2 brings up the whole Nav2 stack, including the MPPI controller, which
  is able to publish /cmd_vel. For localization work — and for the A/B test,
  which drives itself with its own controller — that is both unnecessary weight
  on the Jetson and an unnecessary way for something to command the motors.
  This launch gives the robot a map-frame pose and nothing else.

What it provides:
  * /map                          the static occupancy grid
  * map -> odom TF                published by AMCL (tf_broadcast: true)
  * /amcl_pose                    PoseWithCovarianceStamped in the map frame

Usage:
  ros2 launch yahboomcar_nav x3_localization.launch.py
  ros2 launch yahboomcar_nav x3_localization.launch.py map:=/abs/path/to/x.yaml

Requires the bringup (x3_bringup.launch.py, started by server_x3.py) to already
be running, since AMCL needs /scan_fixed and the odom->base_footprint TF.

Seeding the initial pose:
  AMCL needs a starting guess. Either publish /initialpose (the RViz "2D Pose
  Estimate" tool), or call nav2_client.set_initial_pose(). set_initial_pose_at_
  origin defaults to true here, which seeds the robot at the map origin so that
  localization converges without operator input when the robot is started from
  roughly its mapping start point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('yahboomcar_nav')

    # apartment2 is the map driven AFTER the lidar X-offset was corrected in the
    # URDF, so its geometry is not skewed by the old -0.0115 m guess. It also
    # covers more of the flat (9.00 x 8.40 m vs 8.40 x 7.25 m). Prefer it over
    # `apartment`, which is kept only for comparison.
    default_map = os.path.join(nav_pkg, 'maps', 'apartment2.yaml')
    default_params = os.path.join(nav_pkg, 'params', 'nav2_params_x3.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true) or system clock (false)')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map YAML file to localize against')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameter YAML (supplies the amcl block)')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically configure+activate the lifecycle nodes')

    # ── Static map server ──────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # ── AMCL ───────────────────────────────────────────────────────────────
    # The amcl block in nav2_params_x3.yaml sets OmniMotionModel (correct for a
    # mecanum base; the differential default would localize badly) and consumes
    # /scan_fixed, the constant-beam-count topic the map itself was built from.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── Lifecycle manager ──────────────────────────────────────────────────
    # map_server and amcl are managed nodes: without this they stay UNCONFIGURED
    # and silently publish nothing at all.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_autostart,
        map_server,
        amcl,
        lifecycle_manager,
    ])
