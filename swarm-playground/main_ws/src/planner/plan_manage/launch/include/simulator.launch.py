from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    args = [
        DeclareLaunchArgument('init_x_'),
        DeclareLaunchArgument('init_y_'),
        DeclareLaunchArgument('init_z_'),
        DeclareLaunchArgument('map_size_x_'),
        DeclareLaunchArgument('map_size_y_'),
        DeclareLaunchArgument('map_size_z_'),
        DeclareLaunchArgument('c_num'),
        DeclareLaunchArgument('p_num'),
        DeclareLaunchArgument('min_dist'),
        DeclareLaunchArgument('odometry_topic'),
        DeclareLaunchArgument('drone_id'),
    ]

    drone_id = LaunchConfiguration('drone_id')
    odometry_topic = LaunchConfiguration('odometry_topic')

    nodes = [
        Node(
            package='poscmd_2_odom',
            executable='poscmd_2_odom',
            name=['drone_', drone_id, '_poscmd_2_odom'],
            output='screen',
            parameters=[{
                'init_x': LaunchConfiguration('init_x_'),
                'init_y': LaunchConfiguration('init_y_'),
                'init_z': LaunchConfiguration('init_z_'),
            }],
            remappings=[
                ('~command', ['drone_', drone_id, '_planning/pos_cmd']),
                ('~odometry', ['drone_', drone_id, '_', odometry_topic]),
            ]
        ),
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name=['drone_', drone_id, '_odom_visualization'],
            output='screen',
            parameters=[{
                'color/a': 1.0,
                'color/r': 0.0,
                'color/g': 0.0,
                'color/b': 0.0,
                'covariance_scale': 100.0,
                'robot_scale': 0.35,
                'tf45': False,
                'drone_id': drone_id,
            }],
            remappings=[
                ('~odom', ['drone_', drone_id, '_visual_slam/odom']),
            ]
        ),
        Node(
            package='local_sensing_node',
            executable='pcl_render_node',
            name=['drone_', drone_id, '_pcl_render_node'],
            output='screen',
            parameters=[
                {'sensing_horizon': 5.0,
                 'sensing_rate': 30.0,
                 'estimation_rate': 30.0,
                 'map/x_size': LaunchConfiguration('map_size_x_'),
                 'map/y_size': LaunchConfiguration('map_size_y_'),
                 'map/z_size': LaunchConfiguration('map_size_z_'),
                }
            ],
            remappings=[
                ('~global_map', '/map_generator/global_cloud'),
                ('~odometry', ['/drone_', drone_id, '_', odometry_topic]),
                ('~pcl_render_node/cloud', ['/drone_', drone_id, '_pcl_render_node/cloud']),
            ],
            # Load rosparam YAML file
            parameters=[LaunchConfiguration('camera_yaml', default=['$(find local_sensing_node)/params/camera.yaml'])]
        ),
    ]

    return LaunchDescription(args + nodes)