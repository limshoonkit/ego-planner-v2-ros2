import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('map_processor')
    default_pcd_file_path = os.path.join(package_share_dir, 'maps', 'sample_data3.pcd')
    default_config_file_path = os.path.join(package_share_dir, 'config', 'map_processor_params.yaml')

    return LaunchDescription([
        Node(
            package='map_processor',
            executable='map_processor_node',
            name='map_processor_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            remappings=[
                # ('global_cloud', 'global_map'),
            ],
            parameters=[
                {'pcd_file_path': default_pcd_file_path},
                default_config_file_path,
            ]
        )
    ])