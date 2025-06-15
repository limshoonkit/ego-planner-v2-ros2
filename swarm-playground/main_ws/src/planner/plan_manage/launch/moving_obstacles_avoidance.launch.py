from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	# Arguments
	args = [
		DeclareLaunchArgument('map_size_x', default_value='40.0'),
		DeclareLaunchArgument('map_size_y', default_value='80.0'),
		DeclareLaunchArgument('map_size_z', default_value='3.0'),
		DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'),
	]

	# Map generator node
	map_generator_node = Node(
		package='map_generator',
		executable='random_forest',
		name='random_forest',
		output='screen',
		parameters=[{
			'map/x_size': 20,
			'map/y_size': 30,
			'map/z_size': 3,
			'map/resolution': 0.1,
			'ObstacleShape/seed': 1,
			'map/obs_num': 20,
			'ObstacleShape/lower_rad': 0.5,
			'ObstacleShape/upper_rad': 0.7,
			'ObstacleShape/lower_hei': 0.0,
			'ObstacleShape/upper_hei': 3.0,
			'map/circle_num': 20,
			'ObstacleShape/radius_l': 0.7,
			'ObstacleShape/radius_h': 0.5,
			'ObstacleShape/z_l': 0.7,
			'ObstacleShape/z_h': 0.8,
			'ObstacleShape/theta': 0.5,
			'pub_rate': 1.0,
			'min_distance': 0.8,
		}]
	)

	# Swarm bridge UDP include
	swarm_bridge_launch = IncludeLaunchDescription(
		AnyLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('swarm_bridge'),
				'launch',
				'bridge_udp.launch.py'
			)
		),
		launch_arguments={
			'drone_id': '999',
			'broadcast_ip': '127.0.0.255'
		}.items()
	)

	# Moving obstacles include
	moving_obstacles_launch = IncludeLaunchDescription(
		AnyLaunchDescriptionSource(
			os.path.join(
				get_package_share_directory('moving_obstacles'),
				'launch',
				'obstacle_run.launch.py'
			)
		)
	)

	# Drone configs
	drone_args = [
		# drone_id, init_x, init_y, init_z, target_x, target_y, target_z
		(0,  4, -20, 1,  4,  20, 1),
		(1,  2, -20, 1,  2,  20, 1),
		(2,  0, -20, 1,  0,  20, 1),
		(3, -2, -20, 1, -2,  20, 1),
		(4, -4, -20, 1, -4,  20, 1),
		(5,  4,  20, 1,  4, -20, 1),
		(6,  2,  20, 1,  2, -20, 1),
		(7,  0,  20, 1,  0, -20, 1),
		(8, -2,  20, 1, -2, -20, 1),
		(9, -4,  20, 1, -4, -20, 1),
	]

	drones = []
	for drone_id, init_x, init_y, init_z, target_x, target_y, target_z in drone_args:
		drones.append(
			IncludeLaunchDescription(
				AnyLaunchDescriptionSource(
					os.path.join(
						get_package_share_directory('ego_planner'),
						'launch',
						'include',
						'run_in_sim.launch.py'
					)
				),
				launch_arguments={
					'drone_id': str(drone_id),
					'point_num': '5',
					'init_x': str(init_x),
					'init_y': str(init_y),
					'init_z': str(init_z),
					'target0_x': str(target_x),
					'target0_y': str(target_y),
					'target0_z': str(target_z),
					'target1_x': str(init_x),
					'target1_y': str(init_y),
					'target1_z': str(init_z),
					'target2_x': str(target_x),
					'target2_y': str(target_y),
					'target2_z': str(target_z),
					'target3_x': str(init_x),
					'target3_y': str(init_y),
					'target3_z': str(init_z),
					'target4_x': str(target_x),
					'target4_y': str(target_y),
					'target4_z': str(target_z),
					'map_size_x': LaunchConfiguration('map_size_x'),
					'map_size_y': LaunchConfiguration('map_size_y'),
					'map_size_z': LaunchConfiguration('map_size_z'),
					'odom_topic': LaunchConfiguration('odom_topic'),
				}.items()
			)
		)

	return LaunchDescription(
		args +
		[swarm_bridge_launch, moving_obstacles_launch, map_generator_node] +
		drones
	)
