"""Independent Livox + FAST-LIO diagnostic; never publishes production TF."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    driver = Path(get_package_share_directory('livox_ros_driver2'))
    return LaunchDescription([
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_shadow_driver', output='screen', parameters=[{
                 'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'user_config_path': str(driver / 'config/MID360s_config.json'),
                 'cmdline_input_bd_code': 'livox0000000001',
                 'lvx_file_path': '/tmp/unused.lvx',
             }]),
        Node(package='fast_lio', executable='fastlio_mapping',
             name='fastlio_shadow', output='screen',
             parameters=[str(root / 'config/fastlio/mid360s_shadow.yaml')],
             remappings=[('/Odometry', '/fastlio/odom'),
                         ('/tf', '/fastlio/tf'), ('/tf_static', '/fastlio/tf_static'),
                         ('/path', '/fastlio/path'),
                         ('/cloud_registered', '/fastlio/cloud_world'),
                         ('/cloud_registered_body', '/fastlio/cloud_body'),
                         ('/cloud_effected', '/fastlio/effective_points'),
                         ('/Laser_map', '/fastlio/map')]),
    ])
