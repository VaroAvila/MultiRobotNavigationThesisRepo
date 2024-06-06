cmake_minimum_required(VERSION 3.5)
project(neo_sick_s300-2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
    common/include
)

add_library(${PROJECT_NAME} src/SickS300Receiver.cpp)

add_executable(neo_sick_s300_node src/neo_sick_s300_node.cpp)
add_executable(neo_scan_filter_node src/neo_scan_filter_node.cpp)

ament_export_include_directories(include)

ament_target_dependencies(neo_sick_s300_node
  rclcpp
  sensor_msgs
 )
ament_target_dependencies(neo_scan_filter_node
  rclcpp
  sensor_msgs 
 )

target_link_libraries(neo_sick_s300_node ${PROJECT_NAME})

target_link_libraries(neo_scan_filter_node ${PROJECT_NAME})


install(DIRECTORY include/
   DESTINATION include/
)

install(TARGETS neo_sick_s300_node neo_scan_filter_node
DESTINATION lib/${PROJECT_NAME}
	)

install(DIRECTORY launch
  	
  DESTINATION share/${PROJECT_NAME})

ament_package()