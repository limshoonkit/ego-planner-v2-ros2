from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    drone_id = LaunchConfiguration('drone_id', default=0)
    map_size_x = LaunchConfiguration('map_size_x', default = 200.0)
    map_size_y = LaunchConfiguration('map_size_y', default = 200.0)
    map_size_z = LaunchConfiguration('map_size_z', default = 3.0)
    odom_topic = LaunchConfiguration('odom_topic', default = 'visual_slam/odom')

    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')
    odom_topic_cmd = DeclareLaunchArgument('odom_topic', default_value=odom_topic, description='Odometry topic')

    # Map generator node
    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 100},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 100},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8}
        ]
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
            'drone_id': drone_id,
            'init_x': str(-15.0),
            'init_y': str(0.0),
            'init_z': str(1.0),
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'flight_type': str(2),
            'point_num': str(4),
            'target0_x': str(0.0),
            'target0_y': str(8.0),
            'target0_z': str(1.0),
            'target1_x': str(8.0),
            'target1_y': str(0.0),
            'target1_z': str(1.0),
            'target2_x': str(0.0),
            'target2_y': str(-8.0),
            'target2_z': str(1.0),
            'target3_x': str(-8.0),
            'target3_y': str(0.0),
            'target3_z': str(1.0),
        }.items()
    )

    ld = LaunchDescription()
        
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(odom_topic_cmd)
    ld.add_action(drone_id_cmd)

    ld.add_action(map_generator_node)
    ld.add_action(include_run_in_sim)

    return ld