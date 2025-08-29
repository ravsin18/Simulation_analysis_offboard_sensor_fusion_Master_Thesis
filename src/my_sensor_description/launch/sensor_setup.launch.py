from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'urdf', 'lidar_camera_robot.urdf'])
    rviz_config = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'rviz', 'sensor_config.rviz'])
    world_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'worlds', 'test_world_human.world'])
    gazebo_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': use_sim_time
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'publish_default_positions': True}]
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_path,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'my_robot',
                '-x', '0', '-y', '0', '-z', '0'
            ]
        ),

        # Start RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Voxel Filter Node
        Node(
            package='my_sensor_description',
            executable='voxel_filter_node',
            output='screen',
            parameters=[{'leaf_size': 0.4}]
        ),

        # Point Cloud Merger
'''        Node(
            package='my_sensor_description',
            executable='point_cloud_merger_2.py',
            name='point_cloud_merger',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_topic': '/merged_sensor_points',
                'output_frame': 'map',
                'input_topics': [
                    '/lidar_robot/lidar_robot_scan/out',
                    '/camera_robot_1/camera_robot_sensor_1/downsampled',
                    '/camera_robot/camera_robot_sensor/downsampled'
                ]
            }]
        ),
'''
        # Octomap Server
'''        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[('/cloud_in', '/merged_sensor_points')],
            parameters=[{
                'frame_id': 'map',
                'filter_ground_plane': False,
                'input_frame': 'map',
                'color_mode=true'
                'use_sim_time': use_sim_time,
                'input_cloud_topic': '/merged_sensor_points',
                'resolution': 0.5,
                'sensor_model': {
                    'max_range': 10.0,
                    'hit': 0.7,
                    'miss': 0.4
                },
                'color_factor': 1.0,
                'latch': False,
                'debug': True
            }]
        )
    ])'''
    Node(
        package='my_sensor_description',
        executable='octomap_raycasting_node.py',
        name='octomap_raycasting_node',
        output='screen',
        parameters=[{
            'input_topics': [
                '/lidar_robot/lidar_robot_scan/out',
                '/camera_robot/camera_robot_sensor/downsampled',
                '/camera_robot_1/camera_robot_sensor_1/downsampled'
            ],
            'target_frame': 'map',
            'resolution': 0.1
        }]
    )
    ])