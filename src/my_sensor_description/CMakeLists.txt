# This is a new, complete, and corrected CMakeLists.txt file for your package.
# It compiles all C++ nodes and installs all Python scripts and resources correctly.

cmake_minimum_required(VERSION 3.8)
project(my_sensor_description)

# Set the C++ standard and enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find all necessary ROS 2 and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(my_sensor_description_interfaces REQUIRED)


# --- Define C++ Executables and their Dependencies ---

# 1. Original Simple Evaluator
add_executable(octomap_evaluator_node src/map_evaluator_node.cpp)
ament_target_dependencies(octomap_evaluator_node
  rclcpp
  octomap_msgs
  octomap
  visualization_msgs
)
target_include_directories(octomap_evaluator_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(octomap_evaluator_node ${OpenCV_LIBS})

# 2. Advanced Evaluator (from map_evaluator_node_1.cpp)
add_executable(advanced_octomap_evaluator src/map_evaluator_node_1.cpp)
ament_target_dependencies(advanced_octomap_evaluator
  rclcpp
  octomap_msgs
  octomap
  visualization_msgs
  pcl_conversions
  sensor_msgs
  nav_msgs
  my_sensor_description_interfaces
)

target_include_directories(advanced_octomap_evaluator PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(advanced_octomap_evaluator
  ${PCL_LIBRARIES}
)

# 3. Live Mapping Node
add_executable(octomap_raycasting_node_1 src/octomap_raycasting_node_1.cpp)
ament_target_dependencies(octomap_raycasting_node_1
  rclcpp
  sensor_msgs
  octomap
  octomap_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  tf2_geometry_msgs
)



# --- Add this block for the new Noise Injector Node ---
add_executable(noise_injector_node src/noise_injector_node.cpp)
ament_target_dependencies(noise_injector_node
  rclcpp
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)
target_include_directories(octomap_raycasting_node_1 PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(octomap_raycasting_node_1
  ${PCL_LIBRARIES}
)



# --- Installation Rules ---

# 1. Install all C++ executables
install(TARGETS
  octomap_evaluator_node
#  octomap_raycasting_node
  octomap_raycasting_node_1
  noise_injector_node
  advanced_octomap_evaluator
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install all Python scripts as executables
# This single block correctly handles all scripts without using RENAME.


# 3. Install all other resource directories
install(DIRECTORY
  urdf
  launch
  rviz
  worlds
  nodes
  maps
  config
  DESTINATION share/${PROJECT_NAME}/

)

# === Install Python Scripts ===
install(PROGRAMS
  scripts/point_cloud_merger.py
  scripts/point_cloud_merger_1.py
  scripts/point_cloud_merger_2.py
  scripts/grid_map_fusion_node.py
  scripts/save_map_script.py
  scripts/voxel_filter_node.py  # Make sure this is in the list
  scripts/pcd_publisher_node.py
  scripts/check_python_env.py
  DESTINATION lib/${PROJECT_NAME}
)

# Final step for ROS 2 packaging
ament_package()
