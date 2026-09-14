"""Explicitly started diagnostic localization; never included in robot bringup."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_path', description='Absolute path to connected diagnostic map.npz'),
        DeclareLaunchArgument('cloud_topic', default_value='/pointcloud_raw'),
        Node(package='yahboomcar_bringup', executable='lidar_localizer_node',
             name='lidar_localizer', output='screen',
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
             parameters=[{'map_path': LaunchConfiguration('map_path'),
                          'cloud_topic': LaunchConfiguration('cloud_topic')}]),
    ])
