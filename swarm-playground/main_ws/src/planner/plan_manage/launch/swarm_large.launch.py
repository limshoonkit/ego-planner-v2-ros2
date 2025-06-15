from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths (update as needed)
    ego_planner_launch_dir = os.path.join(
        get_package_share_directory('ego_planner'), 'launch', 'include'
    )
    swarm_bridge_launch_dir = os.path.join(
        get_package_share_directory('swarm_bridge'), 'launch'
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value='72.0'))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value='30.0'))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value='5.0'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'))

    # Swarm bridge include
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(swarm_bridge_launch_dir, 'bridge_udp.launch.py')
            ),
            launch_arguments={
                'drone_id': '999',
                'broadcast_ip': '127.0.0.255'
            }.items()
        )
    )

    # Map generator node
    ld.add_action(
        Node(
            package='map_generator',
            executable='random_forest',
            name='random_forest',
            output='screen',
            parameters=[{
                'map/x_size': 66,
                'map/y_size': 30,
                'map/z_size': 3,
                'map/resolution': 0.1,
                'ObstacleShape/seed': 1,
                'map/obs_num': 300,
                'ObstacleShape/lower_rad': 0.5,
                'ObstacleShape/upper_rad': 0.7,
                'ObstacleShape/lower_hei': 0.0,
                'ObstacleShape/upper_hei': 3.0,
                'map/circle_num': 200,
                'ObstacleShape/radius_l': 0.7,
                'ObstacleShape/radius_h': 0.5,
                'ObstacleShape/z_l': 0.7,
                'ObstacleShape/z_h': 0.8,
                'ObstacleShape/theta': 0.5,
                'sensing/rate': 1.0,
                'min_distance': 1.2,
            }]
        )
    )

    # Drone spawns
    drones = []
    # Drones 0-20 (left to right)
    for i in range(21):
        drones.append({
            'drone_id': str(i),
            'init_x': '-35.0',
            'init_y': str(-10.0 + i),
            'init_z': '0.1',
            'target0_x': '35.0',
            'target0_y': str(-10.0 + i),
            'target0_z': '1'
        })
    # Drones 21-41 (right to left)
    for i in range(21):
        drones.append({
            'drone_id': str(21 + i),
            'init_x': '35.0',
            'init_y': str(-10.0 + i),
            'init_z': '0.1',
            'target0_x': '-35.0',
            'target0_y': str(-10.0 + i),
            'target0_z': '1'
        })

    for drone in drones:
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ego_planner_launch_dir, 'run_in_sim.launch.py')
                ),
                launch_arguments={
                    'drone_id': drone['drone_id'],
                    'init_x': drone['init_x'],
                    'init_y': drone['init_y'],
                    'init_z': drone['init_z'],
                    'target0_x': drone['target0_x'],
                    'target0_y': drone['target0_y'],
                    'target0_z': drone['target0_z'],
                    'map_size_x': LaunchConfiguration('map_size_x'),
                    'map_size_y': LaunchConfiguration('map_size_y'),
                    'map_size_z': LaunchConfiguration('map_size_z'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                }.items()
            )
        )

    return ld