import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_sensor_description')
    
    # Define paths
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'urdf', 'lidar_camera_robot.urdf'])
    rviz_config = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'rviz', 'sensor_config.rviz'])
    #world_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'worlds', 'empty_world.world'])
    #world_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'worlds','office_pillars.world'])
    world_path = PathJoinSubstitution([FindPackageShare('my_sensor_description'), 'worlds','warehouse_new.world'])
    gazebo_launch = PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
    params_file =  os.path.join(get_package_share_directory('my_sensor_description'), 'config', 'octomap_params.yaml')
    file_path = os.path.join(
            get_package_share_directory('my_sensor_description'),
            'maps',
            #'empty_world.pcd'
            #'office_pillars.pcd'
            'warehouse_new.pcd'
    )


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
            parameters=[{'use_sim_time': use_sim_time}],
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

        # Your existing Voxel Filter Node
        Node(
            package='my_sensor_description',
            executable='voxel_filter_node.py',
            output='screen',
            parameters=[
                {'leaf_size': 0.2},
                {'use_sim_time': use_sim_time}
            ]
        ),
        

        # Your existing C++ Octomap Raycasting Node
        Node(
            package='my_sensor_description',
            executable='octomap_raycasting_node_1', 
            name='octomap_raycasting_node_1',
            output='screen',
            parameters=[params_file]
#            parameters=[{
#                'use_sim_time': use_sim_time,
#                'input_topics': [
#                    '/lidar_robot/lidar_robot_scan/out',
#                    '/camera_robot/camera_robot_sensor/downsampled',
#                    '/camera_robot_1/camera_robot_sensor_1/downsampled',
#                    '/lidar_robot_1/lidar_robot_scan/out'
#                ],
#                'target_frame': 'map',
#                'resolution': 0.1
#            },
#            'noisy_sensors'
#            ]
            
        ),

        Node(
            package='my_sensor_description',
            executable='pcd_publisher_node.py',
            name='pcd_publisher',
            output='screen',
            parameters=[{
                'pcd_file': file_path,
                'frame_id': 'map',
                'publish_rate': 100.0  # seconds
            }],
            remappings=[
                ('/cloud_pcd', '/ground_truth_point_cloud')
            ]
        ),
        

        # --- ADDED: The Advanced Evaluator Node ---
        # This node listens to the live map and compares it to the ground truth
#        Node(
#            package='my_sensor_description',
#            executable='advanced_octomap_evaluator',
#            name='advanced_octomap_evaluator',
#            output='screen',
#            parameters=[{
#                'use_sim_time': use_sim_time,
#                # This tells the evaluator where to find the "answer key"
#                'ground_truth_filename': ground_truth_map_path
#            }]
#        ),
    ])