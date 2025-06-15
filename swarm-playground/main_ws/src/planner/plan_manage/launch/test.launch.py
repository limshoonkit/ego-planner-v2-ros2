from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        'map_size_x_', 'map_size_y_', 'map_size_z_',
        'odometry_topic', 'camera_pose_topic', 'depth_topic', 'cloud_topic',
        'cx', 'cy', 'fx', 'fy',
        'max_vel', 'max_acc', 'max_jer', 'planning_horizon',
        'point_num',
        'point0_x', 'point0_y', 'point0_z',
        'point1_x', 'point1_y', 'point1_z',
        'point2_x', 'point2_y', 'point2_z',
        'point3_x', 'point3_y', 'point3_z',
        'point4_x', 'point4_y', 'point4_z',
        'flight_type', 'use_multitopology_trajs', 'drone_id'
    ]

    declare_args = [DeclareLaunchArgument(arg) for arg in args]

    drone_id = LaunchConfiguration('drone_id')
    odometry_topic = LaunchConfiguration('odometry_topic')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')

    node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=[ 'drone_', drone_id, '_ego_planner_node' ],
        output='screen',
        remappings=[
            ('~odom_world', [ '/drone_', drone_id, '_', odometry_topic ]),
            ('~mandatory_stop', '/mandatory_stop_to_planner'),
            ('~planning/trajectory', [ '/drone_', drone_id, '_planning/trajectory' ]),
            ('~planning/data_display', [ '/drone_', drone_id, '_planning/data_display' ]),
            ('~planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
            ('~planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
            ('~planning/heartbeat', [ '/drone_', drone_id, '_traj_server/heartbeat' ]),
            ('/goal', '/goal_with_id'),
            ('~grid_map/odom', [ '/drone_', drone_id, '_', odometry_topic ]),
            ('~grid_map/cloud', [ '/drone_', drone_id, '_', cloud_topic ]),
            ('~grid_map/pose', [ '/drone_', drone_id, '_', camera_pose_topic ]),
            ('~grid_map/depth', [ '/drone_', drone_id, '_', depth_topic ]),
        ],
        parameters=[{
            # planning fsm
            'fsm/flight_type': LaunchConfiguration('flight_type'),
            'fsm/thresh_replan_time': 0.1,
            'fsm/planning_horizon': LaunchConfiguration('planning_horizon'),
            'fsm/emergency_time': 1.0,
            'fsm/realworld_experiment': False,
            'fsm/fail_safe': True,
            'fsm/waypoint_num': LaunchConfiguration('point_num'),
            'fsm/waypoint0_x': LaunchConfiguration('point0_x'),
            'fsm/waypoint0_y': LaunchConfiguration('point0_y'),
            'fsm/waypoint0_z': LaunchConfiguration('point0_z'),
            'fsm/waypoint1_x': LaunchConfiguration('point1_x'),
            'fsm/waypoint1_y': LaunchConfiguration('point1_y'),
            'fsm/waypoint1_z': LaunchConfiguration('point1_z'),
            'fsm/waypoint2_x': LaunchConfiguration('point2_x'),
            'fsm/waypoint2_y': LaunchConfiguration('point2_y'),
            'fsm/waypoint2_z': LaunchConfiguration('point2_z'),
            'fsm/waypoint3_x': LaunchConfiguration('point3_x'),
            'fsm/waypoint3_y': LaunchConfiguration('point3_y'),
            'fsm/waypoint3_z': LaunchConfiguration('point3_z'),
            'fsm/waypoint4_x': LaunchConfiguration('point4_x'),
            'fsm/waypoint4_y': LaunchConfiguration('point4_y'),
            'fsm/waypoint4_z': LaunchConfiguration('point4_z'),
            # grid map
            'grid_map/resolution': 0.1,
            'grid_map/map_size_x': LaunchConfiguration('map_size_x_'),
            'grid_map/map_size_y': LaunchConfiguration('map_size_y_'),
            'grid_map/map_size_z': LaunchConfiguration('map_size_z_'),
            'grid_map/local_update_range_x': 5.5,
            'grid_map/local_update_range_y': 5.5,
            'grid_map/local_update_range_z': 4.5,
            'grid_map/obstacles_inflation': 0.1,
            'grid_map/local_map_margin': 10,
            'grid_map/ground_height': -0.01,
            'grid_map/cx': LaunchConfiguration('cx'),
            'grid_map/cy': LaunchConfiguration('cy'),
            'grid_map/fx': LaunchConfiguration('fx'),
            'grid_map/fy': LaunchConfiguration('fy'),
            'grid_map/use_depth_filter': True,
            'grid_map/depth_filter_tolerance': 0.15,
            'grid_map/depth_filter_maxdist': 5.0,
            'grid_map/depth_filter_mindist': 0.2,
            'grid_map/depth_filter_margin': 2,
            'grid_map/k_depth_scaling_factor': 1000.0,
            'grid_map/skip_pixel': 2,
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
            # manager
            'manager/max_vel': LaunchConfiguration('max_vel'),
            'manager/max_acc': LaunchConfiguration('max_acc'),
            'manager/polyTraj_piece_length': 1.5,
            'manager/feasibility_tolerance': 0.05,
            'manager/planning_horizon': LaunchConfiguration('planning_horizon'),
            'manager/use_multitopology_trajs': LaunchConfiguration('use_multitopology_trajs'),
            'manager/drone_id': drone_id,
            # optimization
            'optimization/constraint_points_perPiece': 5,
            'optimization/weight_obstacle': 10000.0,
            'optimization/weight_obstacle_soft': 5000.0,
            'optimization/weight_swarm': 10000.0,
            'optimization/weight_feasibility': 10000.0,
            'optimization/weight_sqrvariance': 10000.0,
            'optimization/weight_time': 10.0,
            'optimization/obstacle_clearance': 0.1,
            'optimization/obstacle_clearance_soft': 0.5,
            'optimization/swarm_clearance': 0.5,
            'optimization/max_vel': LaunchConfiguration('max_vel'),
            'optimization/max_acc': LaunchConfiguration('max_acc'),
            'optimization/max_jer': LaunchConfiguration('max_jer'),
            'optimization/record_opt': True,
        }]
    )

    return LaunchDescription(declare_args + [node])
