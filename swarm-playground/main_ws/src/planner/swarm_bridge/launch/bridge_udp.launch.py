from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  drone_id = LaunchConfiguration('drone_id')
  broadcast_ip = LaunchConfiguration('broadcast_ip')

  return LaunchDescription([
    DeclareLaunchArgument('drone_id'),
    DeclareLaunchArgument('broadcast_ip'),

    Node(
      package='swarm_bridge',
      executable='bridge_node_udp',
      name=lambda context: f"drone_{context.launch_configurations['drone_id']}_bridge_node",
      output='screen',
      remappings=[
        ('~my_odom', '/vins_estimator/imu_propagate'),
        ('/goal_brig2plner', '/goal_with_id'),
      ],
      parameters=[
        {'broadcast_ip': broadcast_ip,
         'drone_id': drone_id,
         'odom_max_freq': 70}
      ]
    ),

    Node(
      package='swarm_bridge',
      executable='traj2odom_node',
      name='traj2odom_node',
      output='screen',
      parameters=[
        {'odom_hz': 30}
      ]
    ),
  ])
