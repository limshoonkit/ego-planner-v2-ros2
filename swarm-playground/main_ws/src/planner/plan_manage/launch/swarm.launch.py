from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    swarm_bridge_launch = os.path.join(
        get_package_share_directory('swarm_bridge'),
        'launch',
        'bridge_udp.launch.py'
    )
    ego_planner_launch = os.path.join(
        get_package_share_directory('ego_planner'),
        'launch',
        'include',
        'run_in_sim.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_size_x', default_value='50.0'),
        DeclareLaunchArgument('map_size_y', default_value='50.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.0'),
        DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'),

        # Swarm bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(swarm_bridge_launch),
            launch_arguments={
                'drone_id': '999',
                'broadcast_ip': '127.0.0.255'
            }.items()
        ),

        # Map generator node
        Node(
            package='map_generator',
            executable='random_forest',
            name='random_forest',
            output='screen',
            parameters=[{
                'map/x_size': 26,
                'map/y_size': 20,
                'map/z_size': 3,
                'map/resolution': 0.1,
                'ObstacleShape/seed': 1,
                'map/obs_num': 100,
                'ObstacleShape/lower_rad': 0.5,
                'ObstacleShape/upper_rad': 0.7,
                'ObstacleShape/lower_hei': 0.0,
                'ObstacleShape/upper_hei': 3.0,
                'map/circle_num': 100,
                'ObstacleShape/radius_l': 0.7,
                'ObstacleShape/radius_h': 0.5,
                'ObstacleShape/z_l': 0.7,
                'ObstacleShape/z_h': 0.8,
                'ObstacleShape/theta': 0.5,
                'pub_rate': 1.0,
                'min_distance': 0.8
            }]
        ),

        # Drones
        *[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ego_planner_launch),
                launch_arguments={
                    'drone_id': str(i),
                    'init_x': '-15.0',
                    'init_y': str(-9.0 + 2.0 * i),
                    'init_z': '0.1',
                    'target0_x': '15.0',
                    'target0_y': str(9.0 - 2.0 * i),
                    'target0_z': '1',
                    'map_size_x': LaunchConfiguration('map_size_x'),
                    'map_size_y': LaunchConfiguration('map_size_y'),
                    'map_size_z': LaunchConfiguration('map_size_z'),
                    'odom_topic': LaunchConfiguration('odom_topic')
                }.items()
            ) for i in range(10)
        ]
    ])
