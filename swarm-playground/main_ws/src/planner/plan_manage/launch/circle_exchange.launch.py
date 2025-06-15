from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	# Arguments
	map_size_x = DeclareLaunchArgument('map_size_x', default_value='40.0')
	map_size_y = DeclareLaunchArgument('map_size_y', default_value='40.0')
	map_size_z = DeclareLaunchArgument('map_size_z', default_value='4.0')
	odom_topic = DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom')

	swarm_bridge_launch = os.path.join(
		get_package_share_directory('swarm_bridge'), 'launch', 'bridge_udp.launch.py'
	)
	run_in_sim_launch = os.path.join(
		get_package_share_directory('ego_planner'), 'launch', 'include', 'run_in_sim.launch.py'
	)

	# Map generator node
	map_generator_node = Node(
		package='map_generator',
		executable='random_forest',
		name='random_forest',
		output='screen',
		parameters=[{
			'map/x_size': 4,
			'map/y_size': 4,
			'map/z_size': 3,
			'map/resolution': 0.1,
			'ObstacleShape/seed': 1,
			'map/obs_num': 2,
			'ObstacleShape/lower_rad': 0.5,
			'ObstacleShape/upper_rad': 0.7,
			'ObstacleShape/lower_hei': 0.0,
			'ObstacleShape/upper_hei': 3.0,
			'map/circle_num': 1,
			'ObstacleShape/radius_l': 0.7,
			'ObstacleShape/radius_h': 0.5,
			'ObstacleShape/z_l': 0.7,
			'ObstacleShape/z_h': 0.8,
			'ObstacleShape/theta': 0.5,
			'pub_rate': 1.0,
			'min_distance': 0.8,
		}]
	)

	# Swarm bridge include
	swarm_bridge_include = IncludeLaunchDescription(
		AnyLaunchDescriptionSource(swarm_bridge_launch),
		launch_arguments={
			'drone_id': '999',
			'broadcast_ip': '127.0.0.255'
		}.items()
	)

	# Drone configs
	drones = [
		# drone_id, init_x, init_y, init_z, target0_x, target0_y, target0_z
		(0, "-3.184307752023276e-05", "-11.999999999957751", "1.5", "3.184307752023276e-05", "11.999999999957751", "1.5"),
		(1, "-7.05344363676648", "-9.70818895896441", "1.5", "7.05344363676648", "9.70818895896441", "1.5"),
		(2, "-11.412684099558643", "-3.7081857617548453", "1.5", "11.412684099558643", "3.7081857617548453", "1.5"),
		(3, "-11.41267425951457", "3.7082160463238285", "1.5", "11.41267425951457", "-3.7082160463238285", "1.5"),
		(4, "-7.053417875190513", "9.70820767587627", "1.5", "7.053417875190513", "-9.70820767587627", "1.5"),
		(5, "0.0", "12.0", "1.5", "-0.0", "-12.0", "1.5"),
		(6, "7.053417875190508", "9.708207675876276", "1.5", "-7.053417875190508", "-9.708207675876276", "1.5"),
		(7, "11.412674259514569", "3.7082160463238365", "1.5", "-11.412674259514569", "-3.7082160463238365", "1.5"),
		(8, "11.412684099558643", "-3.7081857617548404", "1.5", "-11.412684099558643", "3.7081857617548404", "1.5"),
		(9, "7.05344363676648", "-9.70818895896441", "1.5", "-7.05344363676648", "9.70818895896441", "1.5"),
	]

	drone_includes = []
	for drone in drones:
		drone_id, init_x, init_y, init_z, target0_x, target0_y, target0_z = drone
		drone_includes.append(
			IncludeLaunchDescription(
				AnyLaunchDescriptionSource(run_in_sim_launch),
				launch_arguments={
					'drone_id': str(drone_id),
					'init_x': init_x,
					'init_y': init_y,
					'init_z': init_z,
					'target0_x': target0_x,
					'target0_y': target0_y,
					'target0_z': target0_z,
					'map_size_x': LaunchConfiguration('map_size_x'),
					'map_size_y': LaunchConfiguration('map_size_y'),
					'map_size_z': LaunchConfiguration('map_size_z'),
					'odom_topic': LaunchConfiguration('odom_topic'),
				}.items()
			)
		)

	return LaunchDescription([
		map_size_x,
		map_size_y,
		map_size_z,
		odom_topic,
		swarm_bridge_include,
		map_generator_node,
		*drone_includes
	])