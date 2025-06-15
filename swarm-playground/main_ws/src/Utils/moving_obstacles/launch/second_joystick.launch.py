from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  joy_id = LaunchConfiguration('joy_id')

  return LaunchDescription([
    DeclareLaunchArgument('joy_id', default_value='1'),

    Node(
      package='joy',
      executable='joy_node',
      name=['joy_node_', joy_id],
      remappings=[('joy', ['/joy', joy_id])],
      parameters=[{
        'dev': ['/dev/input/js', joy_id],
        'deadzone': 0.1,
        'autorepeat_rate': 10.0,
        'coalesce_interval': 0.1
      }]
    ),

    Node(
      package='moving_obstacles',
      executable='moving_obstacles',
      name=['moving_obstacles_joy', joy_id],
      output='screen',
      remappings=[('~joy', ['/joy', joy_id])],
      parameters=[{
        'obstacle1_id': 22,
        'obstacle1_init_pos': [10, 2],
        'desired_clearance1': 0.5,
        'obstacle2_id': 23,
        'obstacle2_init_pos': [10, -2],
        'desired_clearance2': 0.5
      }]
    ),

    Node(
      package='odom_visualization',
      executable='odom_visualization',
      name=['fly_car1_odom_visualization_joy', joy_id],
      output='screen',
      remappings=[('~odom', ['/moving_obstacles_joy', joy_id, '/odom_obs1'])],
      parameters=[{
        'mesh_resource': 'package://odom_visualization/meshes/car.dae',
        'color': {'a': 1.0, 'r': 1.0, 'g': 1.0, 'b': 0.0},
        'covariance_scale': 100.0,
        'robot_scale': 0.2,
        'rotate_yaw_deg': 90.0,
        'cross_config': 'false',
        'tf45': False,
        'drone_id': 20
      }]
    ),

    Node(
      package='odom_visualization',
      executable='odom_visualization',
      name=['fly_car2_odom_visualization_joy', joy_id],
      output='screen',
      remappings=[('~odom', ['moving_obstacles_joy', joy_id, '/odom_obs2'])],
      parameters=[{
        'mesh_resource': 'package://odom_visualization/meshes/car.dae',
        'color': {'a': 1.0, 'r': 1.0, 'g': 1.0, 'b': 0.0},
        'covariance_scale': 100.0,
        'robot_scale': 0.2,
        'rotate_yaw_deg': 90.0,
        'cross_config': 'false',
        'tf45': False,
        'drone_id': 21
      }]
    ),
  ])
