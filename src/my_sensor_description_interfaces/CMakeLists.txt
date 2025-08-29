cmake_minimum_required(VERSION 3.8)
project(my_sensor_description_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartTimer.srv"
)

ament_package()
