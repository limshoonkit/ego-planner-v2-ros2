from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
  rviz_config = os.path.join(
    os.getenv('AMENT_PREFIX_PATH').split(':')[0],
    'share', 'ego_planner', 'launch', 'include', 'default.rviz'
  )

  return LaunchDescription([
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz',
      arguments=['-d', rviz_config],
      output='screen'
    )
  ])
