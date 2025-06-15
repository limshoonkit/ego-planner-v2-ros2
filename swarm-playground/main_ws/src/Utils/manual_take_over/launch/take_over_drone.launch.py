from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  drone_id = LaunchConfiguration('drone_id')
  cmd_topic = LaunchConfiguration('cmd_topic')

  return LaunchDescription([
    DeclareLaunchArgument('drone_id', default_value='999'),
    DeclareLaunchArgument('cmd_topic', default_value='/position_cmd'),
    Node(
      package='manual_take_over',
      executable='manual_take_over',
      name=[ 'drone_', drone_id, '_manual_take_over' ],
      output='screen',
      remappings=[
        ('/position_cmd', cmd_topic)
      ]
    )
  ])