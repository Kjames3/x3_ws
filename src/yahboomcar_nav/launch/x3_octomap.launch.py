"""x3_octomap.launch.py — accumulate the tilting lidar's scans into a 3D map.

Consumes /pointcloud_raw from lidar_3d_processor_node and publishes:

    /octomap_binary               compact octree, republished on every insert
    /octomap_binary_throttled     the same tree at 1 Hz — what RViz on the
                                  laptop should use (the raw topic is
                                  ~4.6 Mbit/s and will take the WiFi down)
    /octomap_point_cloud_centers  one point per occupied voxel (PointCloud2)
    /occupied_cells_vis_array     MarkerArray of voxel cubes (heavy)
    /projected_map                the octree flattened to a 2D OccupancyGrid

Run alongside SLAM (which supplies map->odom):

    ros2 launch yahboomcar_nav x3_slam.launch.py
    ros2 launch yahboomcar_nav x3_octomap.launch.py

Frame note: the default is `odom`, NOT `map`.  `map` only exists while AMCL or
slam_toolbox is actually localized -- with Nav2 up but no initial pose set,
there is no map->odom and octomap_server's message filter silently drops every
cloud, reporting only "queue is full" and never naming the missing frame.
Measured on the robot 2026-08-22: 100% of clouds dropped that way.

For a single stationary sweep `odom` is exactly equivalent and never jumps.
Pass frame_id:=map for multi-station mapping with SLAM running -- but note
octomap inserts voxels at insertion time and never moves them again, so a loop
closure corrects future insertions and not past ones.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'frame_id', default_value='odom',
            description='Frame the octree is accumulated in. odom always '
                        'exists; map only while AMCL/SLAM is localized.'),
        DeclareLaunchArgument(
            'resolution', default_value='0.05',
            description='Voxel edge length in metres. Matches the 2D map.'),
        DeclareLaunchArgument(
            'max_range', default_value='8.0',
            description='Rays longer than this only clear, never mark.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/pointcloud_raw',
            description='PointCloud2 input from lidar_3d_processor_node.'),
        DeclareLaunchArgument(
            'throttle_hz', default_value='1.0',
            description='Rate cap for /octomap_binary_throttled (RViz over '
                        'WiFi). Raw /octomap_binary is ~4.6 Mbit/s at 8 Hz.'),
        DeclareLaunchArgument(
            'tilt_node', default_value='false',
            description='Also start lidar_tilt_node (owns /dev/lx16a).'),
        DeclareLaunchArgument(
            'simulate_tilt', default_value='false',
            description='Run lidar_tilt_node with no servo attached.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Follow /clock instead of the wall clock (Gazebo).'),
    ]

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': ParameterValue(LaunchConfiguration('resolution'), value_type=float),
            'frame_id': LaunchConfiguration('frame_id'),
            'base_frame_id': 'base_footprint',
            # ROS 2 octomap_server spells these with dots and an underscore
            # after "point" — `sensor_model/max_range` and `pointcloud_min_z`
            # are ROS 1 names, and an undeclared parameter is SILENTLY IGNORED
            # here, so the wrong spelling reads as "no limit" rather than an
            # error.  Verified against `ros2 param list /octomap_server`.
            'sensor_model.max_range': ParameterValue(LaunchConfiguration('max_range'), value_type=float),
            'sensor_model.hit': 0.7,
            'sensor_model.miss': 0.4,
            'sensor_model.min': 0.12,
            'sensor_model.max': 0.97,
            # Clip in the *target* frame: below the floor and above the ceiling
            # is all mis-registration.  -0.05 keeps a genuinely flat floor.
            'point_cloud_min_z': -0.05,
            'point_cloud_max_z': 2.5,
            'occupancy_min_z': -0.05,
            'occupancy_max_z': 2.5,
            # A single-beam TOF lidar at grazing incidence throws isolated
            # false returns; speckle filtering removes free-floating voxels.
            'filter_speckles': True,
            'compress_map': True,
            'latch': False,
            'publish_free_space': False,
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
        ],
    )

    # RViz on the laptop subscribes to the throttled copy; the raw topic stays
    # available for anything running on the robot itself.
    throttle_node = Node(
        package='yahboomcar_bringup',
        executable='octomap_throttle_node',
        name='octomap_throttle',
        output='screen',
        parameters=[{
            'rate_hz': ParameterValue(
                LaunchConfiguration('throttle_hz'), value_type=float),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    tilt_node = Node(
        package='yahboomcar_bringup',
        executable='lidar_tilt_node',
        name='lidar_tilt_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('tilt_node')),
        parameters=[{
            'simulate': ParameterValue(LaunchConfiguration('simulate_tilt'), value_type=bool),
        }],
    )

    return LaunchDescription(
        args + [octomap_server_node, throttle_node, tilt_node])
