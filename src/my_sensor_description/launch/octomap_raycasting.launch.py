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
            # You might need to set use_sim_time for this as well, depending on your setup
            parameters=[{'publish_default_positions': True, 'use_sim_time': use_sim_time}]
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

        # Voxel Filter Node (Assuming this is still needed)
        # If your C++ Octomap node does not need filtered input, you can comment this out.
        Node(
            package='my_sensor_description',
            executable='voxel_filter_node', # This looks like a C++ executable name
            output='screen',
            parameters=[
                {'leaf_size': 0.4},
                {'use_sim_time': use_sim_time}
            ]
        ),

        # Removed the commented-out Python point cloud merger for clarity

        # C++ Octomap Raycasting Node
        Node(
            package='my_sensor_description',
            # This now points to your C++ executable, not the .py script
            executable='octomap_raycasting_node', 
            name='octomap_raycasting_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, # Crucial for TF lookups in simulation
                'input_topics': [
                    '/lidar_robot/lidar_robot_scan/out',
                    '/camera_robot/camera_robot_sensor/downsampled',
                    '/camera_robot_1/camera_robot_sensor_1/downsampled',
                    '/lidar_robot_1/lidar_robot_scan/out'
                ],
                'target_frame': 'map',
                'resolution': 0.25
            }]
        )
    ])