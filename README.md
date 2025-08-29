# Simulation_analysis_offboard_sensor_fusion_Master_Thesis
ROS 2 package for environment mapping and evaluation with sensor fusion
# ROS 2 Sensor Fusion for Environment Mapping

This repository contains the implementation developed for my Master's thesis on  
A Simulative Analysis of Collaborative Sensor Fusion for Enhanced Environmental Mapping via Off-Board Sensor Integration.  

The project demonstrates how fixed cameras and LiDARs can be combined in a ROS 2 workspace to build voxel-based environment maps, evaluate their completeness, and analyze redundancy across different sensor configurations.

---

## 📂 Repository Structure

```text
ros2_sensor_ws/
|-- src/
|   |-- my_sensor_description/           # Main package (launch, config, URDFs, nodes)
|   |-- my_sensor_description_interfaces # Custom interfaces (srv/msg)
|   |-- YOLOv5-ROS, geometry2, etc.      # External dependencies
|   `-- bbox_ex_msgs                     # Message definitions


Key Directories:
my_sensor_description/
|-- launch/      # ROS 2 launch files
|-- config/      # YAML parameter files
|-- maps/        # Ground-truth and generated maps (PCD/BT)
|-- scripts/     # Python nodes
|-- src/         # C++ nodes
|-- urdf/        # Robot description models
|-- rviz/        # RViz visualization configs
|-- worlds/      # Gazebo simulation worlds
|-- package.xml
`-- CMakeLists.txt


Requirements:
ROS 2 Humble

Gazebo Classic

Dependencies: octomap

# Clone repository
git clone https://github.com/<your-username>/<repo-name>.git ros2_sensor_ws/src

# Build workspace
cd ros2_sensor_ws
colcon build
source install/setup.bash

#launch file:
ros2 launch my_sensor_description generate_ground_truth.launch.py
(edit the Gazebo world file, pcd file for the gazebo world, for sensor configuration also edit the URDF file)
#Evaluation of octomap:
ros2 run my_sensor_description advanced_octomap_evaluator   --ros-args   -p use_sim_time:=true   -p ground_truth_filename:=/home/singh/ros2_sensor_ws/src/my_sensor_description/maps/empty_room.bt   -p eval_min_x:=-10.0   -p eval_max_x:=6.0   -p eval_min_y:=-4.25   -p eval_max_y:=4.5   -p eval_min_z:=0.05   -p eval_max_z:=2.0
(use appropriate ground truth file, evaluation area, .bt file)
#Noise Injection:
ros2 run my_sensor_description noise_injector_node --ros-args --params-file ~/ros2_sensor_ws/install/my_sensor_description/share/my_sensor_description/config/noise_injector_params.yaml
(edit yaml file for noise params)
#Generate CSV file:
ros2 service call /start_evaluation_timer my_sensor_description_interfaces/srv/StartTimer "{duration_seconds: 10.0}"


Title: A Simulative Analysis of Collaborative Sensor Fusion for Enhanced Environmental Mapping via Off-Board Sensor Integration
Author: Ravinder Singh
Institution: Deggendorf Institute of Technology, Deggendorf(Germany)
Submission: August 2025
