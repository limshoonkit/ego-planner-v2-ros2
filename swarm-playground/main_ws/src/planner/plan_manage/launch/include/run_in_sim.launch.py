import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id', default=0)
    map_size_x = LaunchConfiguration('map_size_x', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    init_x = LaunchConfiguration('init_x', default=0.0)
    init_y = LaunchConfiguration('init_y', default=0.0)
    init_z = LaunchConfiguration('init_z', default=0.0)
    flight_type = LaunchConfiguration('flight_type', default=2)
    point_num = LaunchConfiguration('point_num', default=1)
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')
    
    # Target configurations
    target0_x = LaunchConfiguration('target0_x', default=0.0)
    target0_y = LaunchConfiguration('target0_y', default=0.0)
    target0_z = LaunchConfiguration('target0_z', default=0.0)
    target1_x = LaunchConfiguration('target1_x', default=0.0)
    target1_y = LaunchConfiguration('target1_y', default=0.0)
    target1_z = LaunchConfiguration('target1_z', default=0.0)
    target2_x = LaunchConfiguration('target2_x', default=0.0)
    target2_y = LaunchConfiguration('target2_y', default=0.0)
    target2_z = LaunchConfiguration('target2_z', default=0.0)
    target3_x = LaunchConfiguration('target3_x', default=0.0)
    target3_y = LaunchConfiguration('target3_y', default=0.0)
    target3_z = LaunchConfiguration('target3_z', default=0.0)
    target4_x = LaunchConfiguration('target4_x', default=0.0)
    target4_y = LaunchConfiguration('target4_y', default=0.0)
    target4_z = LaunchConfiguration('target4_z', default=0.0)

    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id)
    map_size_x_arg = DeclareLaunchArgument('map_size_x', default_value=map_size_x)
    map_size_y_arg = DeclareLaunchArgument('map_size_y', default_value=map_size_y)
    map_size_z_arg = DeclareLaunchArgument('map_size_z', default_value=map_size_z)
    init_x_arg = DeclareLaunchArgument('init_x', default_value=init_x)
    init_y_arg = DeclareLaunchArgument('init_y', default_value=init_y)
    init_z_arg = DeclareLaunchArgument('init_z', default_value=init_z)
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type)
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num)
    
    # Target waypoint arguments
    target0_x_arg = DeclareLaunchArgument('target0_x', default_value=target0_x)
    target0_y_arg = DeclareLaunchArgument('target0_y', default_value=target0_y)
    target0_z_arg = DeclareLaunchArgument('target0_z', default_value=target0_z)
    target1_x_arg = DeclareLaunchArgument('target1_x', default_value=target1_x)
    target1_y_arg = DeclareLaunchArgument('target1_y', default_value=target1_y)
    target1_z_arg = DeclareLaunchArgument('target1_z', default_value=target1_z)
    target2_x_arg = DeclareLaunchArgument('target2_x', default_value=target2_x)
    target2_y_arg = DeclareLaunchArgument('target2_y', default_value=target2_y)
    target2_z_arg = DeclareLaunchArgument('target2_z', default_value=target2_z)
    target3_x_arg = DeclareLaunchArgument('target3_x', default_value=target3_x)
    target3_y_arg = DeclareLaunchArgument('target3_y', default_value=target3_y)
    target3_z_arg = DeclareLaunchArgument('target3_z', default_value=target3_z)
    target4_x_arg = DeclareLaunchArgument('target4_x', default_value=target4_x)
    target4_y_arg = DeclareLaunchArgument('target4_y', default_value=target4_y)
    target4_z_arg = DeclareLaunchArgument('target4_z', default_value=target4_z)
    
    # Odometry topic argument
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value=odom_topic)
    

    # Get package directories using your provided paths
    ego_planner_launch_dir = os.path.join(
        get_package_share_directory('ego_planner'), 'launch', 'include')
    manual_take_over_launch_dir = os.path.join(
        get_package_share_directory('manual_take_over'), 'launch')

    advanced_param_launch = os.path.join(ego_planner_launch_dir, 'advanced_param.launch.py')
    simulator_launch = os.path.join(ego_planner_launch_dir, 'simulator.launch.py')
    take_over_launch = os.path.join(manual_take_over_launch_dir, 'take_over_drone.launch.py')
    
    # Trajectory server node
    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['/drone_', drone_id, '_planning/pos_cmd']),
            ('planning/trajectory', ['/drone_', drone_id, '_planning/trajectory']),
            ('planning/heartbeat', ['/drone_', drone_id, '_traj_server/heartbeat']),
        ],
        parameters=[{
            'traj_server/time_forward': 1.0
        }]
    )

    advanced_param_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(advanced_param_launch),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            
            # Camera pose and depth topics
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': 'pcl_render_node/cloud',
            
            # Camera intrinsic parameters
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            
            # Motion parameters
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'max_jer': str(20.0),
            'planning_horizon': str(7.5),
            'use_multitopology_trajs': 'False',
            
            # Flight configuration
            'flight_type': flight_type,
            
            # Waypoint parameters
            'point_num': point_num,
            'point0_x': target0_x,
            'point0_y': target0_y,
            'point0_z': target0_z,
            'point1_x': target1_x,
            'point1_y': target1_y,
            'point1_z': target1_z,
            'point2_x': target2_x,
            'point2_y': target2_y,
            'point2_z': target2_z,
            'point3_x': target3_x,
            'point3_y': target3_y,
            'point3_z': target3_z,
            'point4_x': target4_x,
            'point4_y': target4_y,
            'point4_z': target4_z,
        }.items()
    )

    # Include simulator launch
    simulator_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulator_launch),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic,
        }.items()
    )
    
    # Include manual take over launch
    take_over_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(take_over_launch),
        launch_arguments={
            'drone_id': drone_id,
            'cmd_topic': ['/drone_', drone_id, '_planning/pos_cmd'],
        }.items()
    )
    
    return LaunchDescription([
        # Declare all arguments
        drone_id_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        flight_type_arg,
        point_num_arg,
        target0_x_arg,
        target0_y_arg,
        target0_z_arg,
        target1_x_arg,
        target1_y_arg,
        target1_z_arg,
        target2_x_arg,
        target2_y_arg,
        target2_z_arg,
        target3_x_arg,
        target3_y_arg,
        target3_z_arg,
        target4_x_arg,
        target4_y_arg,
        target4_z_arg,
        odom_topic_arg,
        
        # Launch nodes and includes
        advanced_param_launch_include,
        traj_server_node,
        simulator_launch_include,
        take_over_launch_include,
    ])