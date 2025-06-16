#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # LaunchConfigurations
    init_x = LaunchConfiguration('init_x_', default=0.0)
    init_y = LaunchConfiguration('init_y_', default=0.0)
    init_z = LaunchConfiguration('init_z_', default=0.0)
    map_size_x_ = LaunchConfiguration('map_size_x_', default=10.0)
    map_size_y_ = LaunchConfiguration('map_size_y_', default=10.0)
    map_size_z_ = LaunchConfiguration('map_size_z_', default=5.0)
    c_num = LaunchConfiguration('c_num', default=5)
    p_num = LaunchConfiguration('p_num', default=20)
    min_dist = LaunchConfiguration('min_dist', default=1.0)
    odometry_topic = LaunchConfiguration('odometry_topic', default='visual_slam/odom')
    drone_id = LaunchConfiguration('drone_id', default=0)

    # Declare launch arguments
    init_x_arg = DeclareLaunchArgument('init_x_', default_value=init_x, description='Initial X position')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value=init_y, description='Initial Y position')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value=init_z, description='Initial Z position')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x_, description='Map size X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y_, description='Map size Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z_, description='Map size Z')
    c_num_arg = DeclareLaunchArgument('c_num', default_value=c_num, description='Circle number')
    p_num_arg = DeclareLaunchArgument('p_num', default_value=p_num, description='Polygon number')
    min_dist_arg = DeclareLaunchArgument('min_dist', default_value=min_dist, description='Minimum distance')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    # poscmd_2_odom node
    poscmd_2_odom_node = Node(
        package='poscmd_2_odom',
        executable='poscmd_2_odom',
        name=['drone_', drone_id, '_poscmd_2_odom'],
        output='screen',
        parameters=[
            {'init_x': init_x},
            {'init_y': init_y},
            {'init_z': init_z}
        ],
        remappings=[
            ('command', ['drone_', drone_id, '_planning/pos_cmd']),
            ('odometry', ['drone_', drone_id, '_', odometry_topic])
        ],
    )

    # odom_visualization node
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name=['drone_', drone_id, '_odom_visualization'],
        output='screen',
        remappings=[
            ('odom', ['drone_', drone_id, '_visual_slam/odom']),
            ('robot', ['drone_', drone_id, '_vis/robot']),
            ('path', ['drone_', drone_id, '_vis/path']),
            ('time_gap', ['drone_', drone_id, '_vis/time_gap']),
            # ('pose', ['drone_', drone_id, '_vis/pose']),
            # ('velocity', ['drone_', drone_id, '_vis/velocity']),
            # ('covariance', ['drone_', drone_id, '_vis/covariance']),
            # ('covariance_velocity', ['drone_', drone_id, '_vis/covariance_velocity']),
            ('trajectory', ['drone_', drone_id, '_vis/trajectory']),
            ('sensor', ['drone_', drone_id, '_vis/sensor']),
            ('height', ['drone_', drone_id, '_vis/height']),
        ],
        parameters=[
            {'color/a': 1.0},
            {'color/r': 0.0},
            {'color/g': 0.0},
            {'color/b': 0.0},
            {'covariance_scale': 100.0},
            {'robot_scale': 1.0},
            {'tf45': False},
            {'drone_id': drone_id}
        ]
    )

    # Camera parameters from YAML file
    camera_params_file = os.path.join(
        get_package_share_directory('local_sensing'),
        'params',
        'camera.yaml'
    )

    # pcl_render
    pcl_render_node = Node(
        package='local_sensing',
        executable='pcl_render_node',
        name=['drone_', drone_id, '_pcl_render_node'],
        output='screen',
        parameters=[
            camera_params_file,
            {'sensing_horizon': 5.0},
            {'sensing_rate': 30.0},
            {'estimation_rate': 30.0},
            {'map/x_size': map_size_x_},
            {'map/y_size': map_size_y_},
            {'map/z_size': map_size_z_},
        ],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', ['/drone_', drone_id, '_', odometry_topic]),
            ('pcl_render_node/cloud', ['/drone_', drone_id, '_pcl_render_node/cloud']),
        ]
    )

    return LaunchDescription([
        init_x_arg,
        init_y_arg,
        init_z_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        c_num_arg,
        p_num_arg,
        min_dist_arg,
        odometry_topic_arg,
        drone_id_arg,
        poscmd_2_odom_node,
        odom_visualization_node,
        pcl_render_node,
    ])