cmake_minimum_required(VERSION 3.5)
project(sensor_module)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node rclcpp sensor_msgs)

install(TARGETS
    sensor_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
