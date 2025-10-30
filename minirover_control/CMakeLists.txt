cmake_minimum_required(VERSION 3.5)
project(minirover_control)



# Este paquete es solo Python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Usamos ament para Python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


ament_python_install_package(${PROJECT_NAME})

# Marcamos que es un paquete ament
ament_package()
