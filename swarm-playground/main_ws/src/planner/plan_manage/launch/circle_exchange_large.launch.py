from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

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
			'map/x_size': 15,
			'map/y_size': 15,
			'map/z_size': 3,
			'map/resolution': 0.1,
			'ObstacleShape/seed': 1,
			'map/obs_num': 8,
			'ObstacleShape/lower_rad': 0.5,
			'ObstacleShape/upper_rad': 0.7,
			'ObstacleShape/lower_hei': 0.0,
			'ObstacleShape/upper_hei': 3.0,
			'map/circle_num': 5,
			'ObstacleShape/radius_l': 0.7,
			'ObstacleShape/radius_h': 0.5,
			'ObstacleShape/z_l': 0.7,
			'ObstacleShape/z_h': 0.8,
			'ObstacleShape/theta': 0.5,
			'pub_rate': 1.0,
			'min_distance': 0.8
		}]
	)

	# Drone initial positions and targets
	drones = [
		# (drone_id, init_x, init_y, init_z, target0_x, target0_y, target0_z)
		(0, -3.184307752023276e-05, -11.999999999957751, 1.5, 3.184307752023276e-05, 11.999999999957751, 1.5),
		(1, -1.8772434589613818, -11.852255354816934, 1.5, 1.8772434589613818, 11.852255354816934, 1.5),
		(2, -3.708231188598532, -11.412669339462399, 1.5, 3.708231188598532, 11.412669339462399, 1.5),
		(3, -5.4479101133920524, -10.69206600224674, 1.5, 5.4479101133920524, 10.69206600224674, 1.5),
		(4, -7.05344363676648, -9.70818895896441, 1.5, 7.05344363676648, 9.70818895896441, 1.5),
		(5, -8.485298261563804, -8.485264486879728, 1.5, 8.485298261563804, 8.485264486879728, 1.5),
		(6, -9.708217034306568, -7.053404994383904, 1.5, 9.708217034306568, 7.053404994383904, 1.5),
		(7, -10.692087686940049, -5.447867554813076, 1.5, 10.692087686940049, 5.447867554813076, 1.5),
		(8, -11.412684099558643, -3.7081857617548453, 1.5, 11.412684099558643, 3.7081857617548453, 1.5),
		(9, -11.852262826874167, -1.8771962824107544, 1.5, 11.852262826874167, 1.8771962824107544, 1.5),
		(10, -11.999999999989438, 1.5921538760130393e-05, 1.5, 11.999999999989438, -1.5921538760130393e-05, 1.5),
		(11, -11.852257845523544, 1.877227733447808, 1.5, 11.852257845523544, -1.877227733447808, 1.5),
		(12, -11.41267425951457, 3.7082160463238285, 1.5, 11.41267425951457, -3.7082160463238285, 1.5),
		(13, -10.692073230496668, 5.447895927208646, 1.5, 10.692073230496668, -5.447895927208646, 1.5),
		(14, -9.708198317428888, 7.0534307559847, 1.5, 9.708198317428888, -7.0534307559847, 1.5),
		(15, -8.485275745122694, 8.485287003350713, 1.5, 8.485275745122694, -8.485287003350713, 1.5),
		(16, -7.053417875190513, 9.70820767587627, 1.5, 7.053417875190513, -9.70820767587627, 1.5),
		(17, -5.447881741015658, 10.69208045872777, 1.5, 5.447881741015658, -10.69208045872777, 1.5),
		(18, -3.7082009040426023, 11.412679179546652, 1.5, 3.7082009040426023, -11.412679179546652, 1.5),
		(19, -1.8772120079309347, 11.852260336209287, 1.5, 1.8772120079309347, -11.852260336209287, 1.5),
		(20, 0.0, 12.0, 1.5, -0.0, -12.0, 1.5),
		(21, 1.8772120079309293, 11.852260336209289, 1.5, -1.8772120079309293, -11.852260336209289, 1.5),
		(22, 3.708200904042597, 11.412679179546654, 1.5, -3.708200904042597, -11.412679179546654, 1.5),
		(23, 5.447881741015654, 10.692080458727771, 1.5, -5.447881741015654, -10.692080458727771, 1.5),
		(24, 7.053417875190508, 9.708207675876276, 1.5, -7.053417875190508, -9.708207675876276, 1.5),
		(25, 8.485275745122689, 8.485287003350717, 1.5, -8.485275745122689, -8.485287003350717, 1.5),
		(26, 9.708198317428883, 7.053430755984708, 1.5, -9.708198317428883, -7.053430755984708, 1.5),
		(27, 10.692073230496662, 5.447895927208656, 1.5, -10.692073230496662, -5.447895927208656, 1.5),
		(28, 11.412674259514569, 3.7082160463238365, 1.5, -11.412674259514569, -3.7082160463238365, 1.5),
		(29, 11.852257845523543, 1.8772277334478158, 1.5, -11.852257845523543, -1.8772277334478158, 1.5),
		(30, 11.999999999989438, 1.5921538765459464e-05, 1.5, -11.999999999989438, -1.5921538765459464e-05, 1.5),
		(31, 11.852262826874169, -1.877196282410749, 1.5, -11.852262826874169, 1.877196282410749, 1.5),
		(32, 11.412684099558643, -3.7081857617548404, 1.5, -11.412684099558643, 3.7081857617548404, 1.5),
		(33, 10.69208768694005, -5.447867554813071, 1.5, -10.69208768694005, 5.447867554813071, 1.5),
		(34, 9.70821703430657, -7.0534049943839, 1.5, -9.70821703430657, 7.0534049943839, 1.5),
		(35, 8.485298261563804, -8.485264486879728, 1.5, -8.485298261563804, 8.485264486879728, 1.5),
		(36, 7.05344363676648, -9.70818895896441, 1.5, -7.05344363676648, 9.70818895896441, 1.5),
		(37, 5.4479101133920524, -10.69206600224674, 1.5, -5.4479101133920524, 10.69206600224674, 1.5),
		(38, 3.708231188598532, -11.412669339462399, 1.5, -3.708231188598532, 11.412669339462399, 1.5),
		(39, 1.8772434589613818, -11.852255354816934, 1.5, -1.8772434589613818, 11.852255354816934, 1.5),
	]

	# Include run_in_sim.xml for each drone (assuming a Python launch file or XML-to-Python conversion)
	drone_includes = []
	for drone in drones:
		drone_id, init_x, init_y, init_z, target0_x, target0_y, target0_z = drone
		drone_include = IncludeLaunchDescription(
			run_in_sim_launch,
			launch_arguments={
				'drone_id': str(drone_id),
				'init_x': str(init_x),
				'init_y': str(init_y),
				'init_z': str(init_z),
				'target0_x': str(target0_x),
				'target0_y': str(target0_y),
				'target0_z': str(target0_z),
				'map_size_x': LaunchConfiguration('map_size_x'),
				'map_size_y': LaunchConfiguration('map_size_y'),
				'map_size_z': LaunchConfiguration('map_size_z'),
				'odom_topic': LaunchConfiguration('odom_topic'),
			}.items()
		)
		drone_includes.append(drone_include)

	return LaunchDescription([
		map_size_x,
		map_size_y,
		map_size_z,
		odom_topic,
		swarm_bridge_launch,
		map_generator_node,
		*drone_includes
	])
