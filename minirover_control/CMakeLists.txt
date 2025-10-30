cmake_minimum_required(VERSION 3.5)
project(minirover_display)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS
    minirover_display/display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
