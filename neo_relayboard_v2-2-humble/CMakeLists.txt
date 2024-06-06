cmake_minimum_required(VERSION 3.5)
project(neo_relayboard_v2-2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(std_srvs)

include_directories(
    ${Boost_INCLUDE_DIRS}
    common/include
)

ament_export_dependencies(std_msgs tf geometry_msgs nav_msgs sensor_msgs)

ament_export_include_directories(common/include)

add_library(${PROJECT_NAME} common/src/NeoRelayBoardNode.cpp 
  common/src/RelayBoardClient.cpp common/src/SerialIO.cpp common/src/StrUtil.cpp)

ament_export_libraries(${PROJECT_NAME})

add_executable(neo_relayboard_node src/neo_relayboard_v2_node.cpp)

ament_target_dependencies(neo_relayboard_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  neo_msgs2
  neo_srvs2
  std_srvs)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  neo_msgs2
  neo_srvs2
  std_srvs)


target_link_libraries(neo_relayboard_node ${PROJECT_NAME})

install(DIRECTORY common/include/
   DESTINATION common/include/
)

install(TARGETS neo_relayboard_node
DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
    
  DESTINATION share/${PROJECT_NAME})

ament_package()