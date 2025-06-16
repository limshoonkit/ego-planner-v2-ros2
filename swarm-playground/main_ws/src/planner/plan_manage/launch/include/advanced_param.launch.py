from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id', default=0)
    map_size_x = LaunchConfiguration('map_size_x_', default=42.0)
    map_size_y = LaunchConfiguration('map_size_y_', default=30.0)
    map_size_z = LaunchConfiguration('map_size_z_', default=5.0)
    
    odometry_topic = LaunchConfiguration('odometry_topic', default='odom')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic', default='camera_pose')
    depth_topic = LaunchConfiguration('depth_topic', default='depth_image')
    cloud_topic = LaunchConfiguration('cloud_topic', default='cloud')
    
    cx = LaunchConfiguration('cx', default=321.04638671875)
    cy = LaunchConfiguration('cy', default=243.44969177246094)
    fx = LaunchConfiguration('fx', default=387.229248046875)
    fy = LaunchConfiguration('fy', default=387.229248046875)
    
    max_vel = LaunchConfiguration('max_vel', default=2.0)
    max_acc = LaunchConfiguration('max_acc', default=3.0)
    max_jer = LaunchConfiguration('max_jer', default=20.0)
    planning_horizon = LaunchConfiguration('planning_horizon', default=7.5)
    use_multitopology_trajs = LaunchConfiguration('use_multitopology_trajs', default=False)
    
    flight_type = LaunchConfiguration('flight_type', default=2)
    point_num = LaunchConfiguration('point_num', default=1)
    point0_x = LaunchConfiguration('point0_x', default=0.0)
    point0_y = LaunchConfiguration('point0_y', default=0.0)
    point0_z = LaunchConfiguration('point0_z', default=0.0)
    point1_x = LaunchConfiguration('point1_x', default=10.0)
    point1_y = LaunchConfiguration('point1_y', default=10.0)
    point1_z = LaunchConfiguration('point1_z', default=0.0)
    point2_x = LaunchConfiguration('point2_x', default=20.0)
    point2_y = LaunchConfiguration('point2_y', default=20.0)
    point2_z = LaunchConfiguration('point2_z', default=1.0)
    point3_x = LaunchConfiguration('point3_x', default=-10.0)
    point3_y = LaunchConfiguration('point3_y', default=-10.0)
    point3_z = LaunchConfiguration('point3_z', default=1.0)
    point4_x = LaunchConfiguration('point4_x', default=30.0)
    point4_y = LaunchConfiguration('point4_y', default=30.0)
    point4_z = LaunchConfiguration('point4_z', default=1.0)

    drone_id_arg = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')
    map_size_x_arg = DeclareLaunchArgument('map_size_x_', default_value=map_size_x, description='Map size along X')
    map_size_y_arg = DeclareLaunchArgument('map_size_y_', default_value=map_size_y, description='Map size along Y')
    map_size_z_arg = DeclareLaunchArgument('map_size_z_', default_value=map_size_z, description='Map size along Z')
    odometry_topic_arg = DeclareLaunchArgument('odometry_topic', default_value=odometry_topic, description='Odometry topic')
    camera_pose_topic_arg = DeclareLaunchArgument('camera_pose_topic', default_value=camera_pose_topic, description='Camera pose topic')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Depth topic')
    cloud_topic_arg = DeclareLaunchArgument('cloud_topic', default_value=cloud_topic, description='Point cloud topic')
    cx_arg = DeclareLaunchArgument('cx', default_value=cx, description='Camera intrinsic cx')
    cy_arg = DeclareLaunchArgument('cy', default_value=cy, description='Camera intrinsic cy')
    fx_arg = DeclareLaunchArgument('fx', default_value=fx, description='Camera intrinsic fx')
    fy_arg = DeclareLaunchArgument('fy', default_value=fy, description='Camera intrinsic fy')
    max_vel_arg = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Maximum velocity')
    max_acc_arg = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Maximum acceleration')
    max_jer_arg = DeclareLaunchArgument('max_jer', default_value=max_jer, description='Maximum jerk')
    planning_horizon_arg = DeclareLaunchArgument('planning_horizon', default_value=planning_horizon, description='Planning horizon')
    use_multitopology_trajs_arg = DeclareLaunchArgument('use_multitopology_trajs', default_value=use_multitopology_trajs, description='Use multitopology trajectories')
    
    flight_type_arg = DeclareLaunchArgument('flight_type', default_value=flight_type, description='flight_type')
    point_num_arg = DeclareLaunchArgument('point_num', default_value=point_num, description='Number of waypoints')
    point0_x_arg = DeclareLaunchArgument('point0_x', default_value=point0_x, description='Waypoint 0 X coordinate')
    point0_y_arg = DeclareLaunchArgument('point0_y', default_value=point0_y, description='Waypoint 0 Y coordinate')
    point0_z_arg = DeclareLaunchArgument('point0_z', default_value=point0_z, description='Waypoint 0 Z coordinate')
    point1_x_arg = DeclareLaunchArgument('point1_x', default_value=point1_x, description='Waypoint 1 X coordinate')
    point1_y_arg = DeclareLaunchArgument('point1_y', default_value=point1_y, description='Waypoint 1 Y coordinate')
    point1_z_arg = DeclareLaunchArgument('point1_z', default_value=point1_z, description='Waypoint 1 Z coordinate')
    point2_x_arg = DeclareLaunchArgument('point2_x', default_value=point2_x, description='Waypoint 2 X coordinate')
    point2_y_arg = DeclareLaunchArgument('point2_y', default_value=point2_y, description='Waypoint 2 Y coordinate')
    point2_z_arg = DeclareLaunchArgument('point2_z', default_value=point2_z, description='Waypoint 2 Z coordinate')
    point3_x_arg = DeclareLaunchArgument('point3_x', default_value=point3_x, description='Waypoint 3 X coordinate')
    point3_y_arg = DeclareLaunchArgument('point3_y', default_value=point3_y, description='Waypoint 3 Y coordinate')
    point3_z_arg = DeclareLaunchArgument('point3_z', default_value=point3_z, description='Waypoint 3 Z coordinate')
    point4_x_arg = DeclareLaunchArgument('point4_x', default_value=point4_x, description='Waypoint 4 X coordinate')
    point4_y_arg = DeclareLaunchArgument('point4_y', default_value=point4_y, description='Waypoint 4 Y coordinate')
    point4_z_arg = DeclareLaunchArgument('point4_z', default_value=point4_z, description='Waypoint 4 Z coordinate')

    # Create the ego_planner_node
    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', ['/drone_', drone_id, '_', odometry_topic]),
            ('mandatory_stop', '/mandatory_stop_to_planner'),
            ('planning/trajectory', ['/drone_', drone_id, '_planning/trajectory']),
            ('planning/data_display', ['/drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
            ('planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
            ('planning/heartbeat', ['/drone_', drone_id, '_traj_server/heartbeat']),
            ('goal', '/goal_with_id'),
            ('grid_map/odom', ['/drone_', drone_id, '_', odometry_topic]),
            ('grid_map/cloud', ['/drone_', drone_id, '_', cloud_topic]),
            ('grid_map/pose', ['/drone_', drone_id, '_', camera_pose_topic]),
            ('grid_map/depth', ['/drone_', drone_id, '_', depth_topic]),
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

    ld = LaunchDescription()

    # Add LaunchArguments
    ld.add_action(drone_id_arg)
    ld.add_action(map_size_x_arg)
    ld.add_action(map_size_y_arg)
    ld.add_action(map_size_z_arg)
    ld.add_action(odometry_topic_arg)
    ld.add_action(camera_pose_topic_arg)
    ld.add_action(depth_topic_arg)
    ld.add_action(cloud_topic_arg)
    ld.add_action(cx_arg)
    ld.add_action(cy_arg)
    ld.add_action(fx_arg)
    ld.add_action(fy_arg)
    ld.add_action(max_vel_arg)
    ld.add_action(max_acc_arg)
    ld.add_action(max_jer_arg)
    ld.add_action(planning_horizon_arg)
    ld.add_action(use_multitopology_trajs_arg)

    ld.add_action(flight_type_arg)
    ld.add_action(point_num_arg)
    ld.add_action(point0_x_arg)
    ld.add_action(point0_y_arg)
    ld.add_action(point0_z_arg)
    ld.add_action(point1_x_arg)
    ld.add_action(point1_y_arg)
    ld.add_action(point1_z_arg)
    ld.add_action(point2_x_arg)
    ld.add_action(point2_y_arg)
    ld.add_action(point2_z_arg)
    ld.add_action(point3_x_arg)
    ld.add_action(point3_y_arg)
    ld.add_action(point3_z_arg)
    ld.add_action(point4_x_arg)
    ld.add_action(point4_y_arg)
    ld.add_action(point4_z_arg)

    # Add Node
    ld.add_action(ego_planner_node)

    return ld
