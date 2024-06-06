cmake_minimum_required(VERSION 3.5)
project(neo_teleop2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(neo_teleop2_node src/neo_teleop.cpp)

ament_target_dependencies(neo_teleop2_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs)


install(TARGETS neo_teleop2_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    
  DESTINATION share/${PROJECT_NAME})

ament_package()