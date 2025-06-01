cmake_minimum_required(VERSION 3.8)
project(my_sensor_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY urdf launch rviz worlds scripts nodes 
  DESTINATION share/${PROJECT_NAME}/
  
)
install(PROGRAMS
  scripts/point_cloud_merger.py
  scripts/point_cloud_merger_1.py
  scripts/point_cloud_merger_2.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
