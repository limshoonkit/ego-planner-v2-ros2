from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='24')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='24')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='7')
    odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[{
            'map/x_size': 20,
            'map/y_size': 20,
            'map/z_size': 3,
            'map/resolution': 0.1,
            'ObstacleShape/seed': 1,
            'map/obs_num': 122.29299363057325,
            'ObstacleShape/lower_rad': 0.5,
            'ObstacleShape/upper_rad': 0.7,
            'ObstacleShape/lower_hei': 0.0,
            'ObstacleShape/upper_hei': 3.0,
            'map/circle_num': 122.29299363057325,
            'ObstacleShape/radius_l': 0.7,
            'ObstacleShape/radius_h': 0.5,
            'ObstacleShape/z_l': 0.7,
            'ObstacleShape/z_h': 0.8,
            'ObstacleShape/theta': 0.5,
            'pub_rate': 1.0,
            'min_distance': 0.8,
        }]
    )

    ego_planner_dir = os.path.join(
        get_package_share_directory('ego_planner'),
        'launch',
        'include',
        'run_in_sim.launch.py'
    )

    include_run_in_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ego_planner_dir),
        launch_arguments={
            'drone_id': '0',
            'init_x': '-12',
            'init_y': '0',
            'init_z': '1',
            'map_size_x': LaunchConfiguration('map_size_x'),
            'map_size_y': LaunchConfiguration('map_size_y'),
            'map_size_z': LaunchConfiguration('map_size_z'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'point_num': '5',
            'target0_x': '0',
            'target0_y': '8',
            'target0_z': '1',
            'target1_x': '8',
            'target1_y': '0',
            'target1_z': '1',
            'target2_x': '0',
            'target2_y': '-8',
            'target2_z': '1',
            'target3_x': '-8',
            'target3_y': '0',
            'target3_z': '1',
            'target4_x': '0',
            'target4_y': '0',
            'target4_z': '1',
        }.items()
    )

    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        odom_topic,
        map_generator_node,
        include_run_in_sim
    ])