from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='150.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='150.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='2.0')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')

    swarm_bridge_launch = os.path.join(
        get_package_share_directory('swarm_bridge'), 'launch', 'bridge_udp.launch.py'
    )
    run_in_sim_launch = os.path.join(
        get_package_share_directory('ego_planner'), 'launch', 'include', 'run_in_sim.launch.py'
    )

    # Swarm bridge include
    swarm_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(swarm_bridge_launch),
        launch_arguments={
            'drone_id': '999',
            'broadcast_ip': '127.0.0.255'
        }.items()
    )

    # Assign goals node
    assign_goals_node = Node(
        package='assign_goals',
        executable='assign_goals_node',
        name='assign_goals_node',
        output='screen'
    )

    # Map generator node
    map_generator_node = Node(
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
            'map/obs_num': 50,
            'ObstacleShape/lower_rad': 0.5,
            'ObstacleShape/upper_rad': 0.7,
            'ObstacleShape/lower_hei': 0.0,
            'ObstacleShape/upper_hei': 3.0,
            'map/circle_num': 50,
            'ObstacleShape/radius_l': 0.7,
            'ObstacleShape/radius_h': 0.5,
            'ObstacleShape/z_l': 0.7,
            'ObstacleShape/z_h': 0.8,
            'ObstacleShape/theta': 0.5,
            'pub_rate': 1.0,
            'min_distance': 0.8,
        }]
    )

    # Drones
    drones = []
    y_values = [-9.0, -7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0, 9.0]
    for i, y in enumerate(y_values):
        drones.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(run_in_sim_launch),
                launch_arguments={
                    'drone_id': str(i),
                    'init_x': '-15.0',
                    'init_y': str(y),
                    'init_z': '0.1',
                    'map_size_x': LaunchConfiguration('map_size_x'),
                    'map_size_y': LaunchConfiguration('map_size_y'),
                    'map_size_z': LaunchConfiguration('map_size_z'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'flight_type': '1'
                }.items()
            )
        )

    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        odom_topic,
        swarm_bridge,
        assign_goals_node,
        map_generator_node,
        *drones
    ])