cmake_minimum_required(VERSION 3.5)
project(minirover_display)

<<<<<<< HEAD
# Este paquete es solo Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
=======
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
>>>>>>> a150711bad2a1daa83194e6640846a2ec3aee9d4

install(
  PROGRAMS
    minirover_display/display_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
