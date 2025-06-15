from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  station_id_arg = DeclareLaunchArgument(
    'station_id',
    default_value='0',
    description='ID of the ground station'
  )

  station_id = LaunchConfiguration('station_id')

  drone_ips = [
    {'name': f'drone_ip_{i}', 'value': '127.0.0.1'} for i in range(10)
  ]
  drone_ip_params = {ip['name']: ip['value'] for ip in drone_ips}

  ground_station_ip_params = {
    'ground_station_ip_0': '127.0.0.1'
  }

  bridge_node = Node(
    package='swarm_bridge',
    executable='bridge_node_tcp',
    name=lambda context: f'station_{station_id.perform(context)}_bridge_node_tcp',
    output='screen',
    parameters=[{
      'self_id': station_id,
      'is_ground_station': True,
      'drone_num': 10,
      **drone_ip_params,
      'ground_station_num': 1,
      **ground_station_ip_params
    }]
  )

  traj2odom_node = Node(
    package='swarm_bridge',
    executable='traj2odom_node',
    name='traj2odom_node',
    output='screen',
    parameters=[{
      'odom_hz': 30
    }]
  )

  return LaunchDescription([
    station_id_arg,
    bridge_node,
    traj2odom_node
  ])
