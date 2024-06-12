cmake_minimum_required(VERSION 3.5)
project(robot_control_module)

include_directories(
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/../json-develop/include")



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tracetools REQUIRED)
find_package(trajectory_msgs REQUIRED) # 用于JointTrajectory消息
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)   
find_package(sensor_msgs REQUIRED)

add_executable(robot_control_node src/trajectory_publisher_node.cpp src/csv_read.cpp)
#add_executable(robot_control_node src/robot_control_node.cpp src/csv_read.cpp)
#add_executable(robot_control_node src/trajectory_publisher_node_single_point.cpp)


target_link_libraries(robot_control_node
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tracetools_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
    ${trajectory_msgs_LIBRARIES})

  

ament_target_dependencies(robot_control_node
  rclcpp 
  std_msgs 
  geometry_msgs 
  trajectory_msgs
  tracetools
  builtin_interfaces
  control_msgs  
  sensor_msgs
)

install(TARGETS
    robot_control_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
