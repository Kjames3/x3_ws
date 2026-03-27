"""
x3_nav2.launch.py
Unified Nav2 bringup for the Yahboom X3 mecanum robot.

Works for both modes:
  Physical (--ros2):  ros2 launch yahboomcar_nav x3_nav2.launch.py
  Simulation (--sim): ros2 launch yahboomcar_nav x3_nav2.launch.py use_sim_time:=true

Launch arguments:
  use_sim_time  [false]  Set true when running with Gazebo simulation.
  map           [yahboomcar.yaml]  Full path to a ROS map YAML file.
                                   Ignored when slam:=true.
  params_file   [nav2_params_x3.yaml]  Full path to Nav2 parameter file.
  slam          [false]  Set true to run slam_toolbox instead of AMCL.
                         The robot will build/extend the map while navigating.

Nav2 stack used:
  Global planner : SmacPlanner2D  (holonomic-friendly, no heading constraints)
  Smoother       : SavitzkyGolay  (removes grid staircase artefacts)
  Controller     : MPPI with Omni motion model (full vx/vy/wz holonomic)
  Localization   : AMCL (OmniMotionModel) or slam_toolbox
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg    = get_package_share_directory('yahboomcar_nav')
    nav2_pkg   = get_package_share_directory('nav2_bringup')

    default_map    = os.path.join(nav_pkg, 'maps', 'yahboomcar.yaml')
    default_params = os.path.join(nav_pkg, 'params', 'nav2_params_x3.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────
    use_sim_time  = LaunchConfiguration('use_sim_time')
    map_yaml      = LaunchConfiguration('map')
    params_file   = LaunchConfiguration('params_file')
    slam          = LaunchConfiguration('slam')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use Gazebo simulation clock (true) or system clock (false)')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to map YAML file (ignored when slam:=true)')

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to Nav2 parameter YAML file')

    declare_slam = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Run slam_toolbox for live mapping instead of AMCL')

    # ── Nav2 bringup (AMCL mode, slam=false) ───────────────────────────────
    # nav2_bringup's bringup_launch.py handles full lifecycle management.
    # It reads slam:=false/true internally to decide whether to start AMCL
    # and the static map server.
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          map_yaml,
            'use_sim_time': use_sim_time,
            'params_file':  params_file,
            'slam':         slam,
        }.items(),
    )

    return LaunchDescription([
        # Suppress common "waiting for transform" warnings during startup
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_slam,

        nav2_bringup,
    ])
