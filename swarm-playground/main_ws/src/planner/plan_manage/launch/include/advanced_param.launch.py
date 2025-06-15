from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument('drone_id')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic')
    
    # Camera intrinsic parameters
    cx_arg = DeclareLaunchArgument('cx')
    cy_arg = DeclareLaunchArgument('cy')
    fx_arg = DeclareLaunchArgument('fx')
    fy_arg = DeclareLaunchArgument('fy')
    
    # Motion parameters
    max_vel_arg = DeclareLaunchArgument('max_vel')
    max_acc_arg = DeclareLaunchArgument('max_acc')
    max_jer_arg = DeclareLaunchArgument('max_jer')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon')
    use_multitopology_trajs_arg = DeclareLaunchArgument('use_multitopology_trajs')
    
    # Flight parameters
    flight_type_arg = DeclareLaunchArgument('flight_type')
    point_num_arg = DeclareLaunchArgument('point_num')
    
    # Waypoint parameters
    point0_x_arg = DeclareLaunchArgument('point0_x')
    point0_y_arg = DeclareLaunchArgument('point0_y')
    point0_z_arg = DeclareLaunchArgument('point0_z')
    point1_x_arg = DeclareLaunchArgument('point1_x')
    point1_y_arg = DeclareLaunchArgument('point1_y')
    point1_z_arg = DeclareLaunchArgument('point1_z')
    point2_x_arg = DeclareLaunchArgument('point2_x')
    point2_y_arg = DeclareLaunchArgument('point2_y')
    point2_z_arg = DeclareLaunchArgument('point2_z')
    point3_x_arg = DeclareLaunchArgument('point3_x')
    point3_y_arg = DeclareLaunchArgument('point3_y')
    point3_z_arg = DeclareLaunchArgument('point3_z')
    point4_x_arg = DeclareLaunchArgument('point4_x')
    point4_y_arg = DeclareLaunchArgument('point4_y')
    point4_z_arg = DeclareLaunchArgument('point4_z')
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    map_size_x = LaunchConfiguration('map_size_x_')
    map_size_y = LaunchConfiguration('map_size_y_')
    map_size_z = LaunchConfiguration('map_size_z_')
    odometry_topic = LaunchConfiguration('odometry_topic')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    max_jer = LaunchConfiguration('max_jer')
    planning_horizon = LaunchConfiguration('planning_horizon')
    use_multitopology_trajs = LaunchConfiguration('use_multitopology_trajs')
    
    flight_type = LaunchConfiguration('flight_type')
    point_num = LaunchConfiguration('point_num')
    
    point0_x = LaunchConfiguration('point0_x')
    point0_y = LaunchConfiguration('point0_y')
    point0_z = LaunchConfiguration('point0_z')
    point1_x = LaunchConfiguration('point1_x')
    point1_y = LaunchConfiguration('point1_y')
    point1_z = LaunchConfiguration('point1_z')
    point2_x = LaunchConfiguration('point2_x')
    point2_y = LaunchConfiguration('point2_y')
    point2_z = LaunchConfiguration('point2_z')
    point3_x = LaunchConfiguration('point3_x')
    point3_y = LaunchConfiguration('point3_y')
    point3_z = LaunchConfiguration('point3_z')
    point4_x = LaunchConfiguration('point4_x')
    point4_y = LaunchConfiguration('point4_y')
    point4_z = LaunchConfiguration('point4_z')
    
    # Create the ego_planner_node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('~/odom_world', ['/drone_', drone_id, '_', odometry_topic]),
            ('~/mandatory_stop', '/mandatory_stop_to_planner'),
            ('~/planning/trajectory', ['/drone_', drone_id, '_planning/trajectory']),
            ('~/planning/data_display', ['/drone_', drone_id, '_planning/data_display']),
            ('~/planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
            ('~/planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
            ('~/planning/heartbeat', ['/drone_', drone_id, '_traj_server/heartbeat']),
            ('/goal', '/goal_with_id'),
            ('~/grid_map/odom', ['/drone_', drone_id, '_', odometry_topic]),
            ('~/grid_map/cloud', ['/drone_', drone_id, '_', cloud_topic]),
            ('~/grid_map/pose', ['/drone_', drone_id, '_', camera_pose_topic]),
            ('~/grid_map/depth', ['/drone_', drone_id, '_', depth_topic]),
        ],
        parameters=[{
            # Planning FSM parameters
            'fsm/flight_type': flight_type,
            'fsm/thresh_replan_time': 1.0,
            'fsm/planning_horizon': planning_horizon,
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': False,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': point_num,
            'fsm/waypoint0_x': point0_x,
            'fsm/waypoint0_y': point0_y,
            'fsm/waypoint0_z': point0_z,
            'fsm/waypoint1_x': point1_x,
            'fsm/waypoint1_y': point1_y,
            'fsm/waypoint1_z': point1_z,
            'fsm/waypoint2_x': point2_x,
            'fsm/waypoint2_y': point2_y,
            'fsm/waypoint2_z': point2_z,
            'fsm/waypoint3_x': point3_x,
            'fsm/waypoint3_y': point3_y,
            'fsm/waypoint3_z': point3_z,
            'fsm/waypoint4_x': point4_x,
            'fsm/waypoint4_y': point4_y,
            'fsm/waypoint4_z': point4_z,
            
            # Grid map parameters
            'grid_map/resolution': 0.1,
            'grid_map/map_size_x': map_size_x,
            'grid_map/map_size_y': map_size_y,
            'grid_map/map_size_z': map_size_z,
            'grid_map/local_update_range_x': 5.5,
            'grid_map/local_update_range_y': 5.5,
            'grid_map/local_update_range_z': 2.0,
            'grid_map/obstacles_inflation': 0.1,
            'grid_map/local_map_margin': 10,
            'grid_map/enable_virtual_wall': True,
            'grid_map/virtual_ceil': 3.0,
            'grid_map/virtual_ground': -0.1,
            'grid_map/ground_height': -0.01,
            
            # Camera parameters
            'grid_map/cx': cx,
            'grid_map/cy': cy,
            'grid_map/fx': fx,
            'grid_map/fy': fy,
            
            # Depth filter parameters
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.2,
            'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 2,
            
            # Local fusion parameters
            'grid_map/p_hit': 0.65,
            'grid_map/p_miss': 0.35,
            'grid_map/p_min': 0.12,
            'grid_map/p_max': 0.90,
            'grid_map/p_occ': 0.80,
            'grid_map/fading_time': 1000.0,
            'grid_map/min_ray_length': 0.1,
            'grid_map/max_ray_length': 4.5,
            'grid_map/visualization_truncate_height': 1.9,
            'grid_map/show_occ_time': False,
            'grid_map/pose_type': 1,
            'grid_map/frame_id': 'world',
            
            # Planner manager parameters
            'manager/max_vel': max_vel,
            'manager/max_acc': max_acc,
            'manager/polyTraj_piece_length': 1.5,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': planning_horizon,
            'manager/use_multitopology_trajs': use_multitopology_trajs,
            'manager/drone_id': drone_id,
            
            # Trajectory optimization parameters
            'optimization/constraint_points_perPiece': 5,
            'optimization/weight_obstacle': 10000.0,
            'optimization/weight_obstacle_soft': 5000.0,
            'optimization/weight_swarm': 10000.0,
            'optimization/weight_feasibility': 10000.0,
            'optimization/weight_sqrvariance': 10000.0,
            'optimization/weight_time': 10.0,
            'optimization/obstacle_clearance': 0.1,
            'optimization/obstacle_clearance_soft': 0.5,
            'optimization/swarm_clearance': 0.15000000000000002,
            'optimization/max_vel': max_vel,
            'optimization/max_acc': max_acc,
            'optimization/max_jer': max_jer,
            'optimization/record_opt': True,
        }]
    )
    
    return LaunchDescription([
        # Declare all arguments
        drone_id_arg,
        map_size_x_arg,
        map_size_y_arg,
        map_size_z_arg,
        odometry_topic_arg,
        camera_pose_topic_arg,
        depth_topic_arg,
        cloud_topic_arg,
        cx_arg,
        cy_arg,
        fx_arg,
        fy_arg,
        max_vel_arg,
        max_acc_arg,
        max_jer_arg,
        planning_horizon_arg,
        use_multitopology_trajs_arg,
        flight_type_arg,
        point_num_arg,
        point0_x_arg,
        point0_y_arg,
        point0_z_arg,
        point1_x_arg,
        point1_y_arg,
        point1_z_arg,
        point2_x_arg,
        point2_y_arg,
        point2_z_arg,
        point3_x_arg,
        point3_y_arg,
        point3_z_arg,
        point4_x_arg,
        point4_y_arg,
        point4_z_arg,
        
        # Launch the node
        ego_planner_node,
    ])