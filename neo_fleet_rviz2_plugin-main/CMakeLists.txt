cmake_minimum_required(VERSION 3.5)
project(neo_fleet_rviz2_plugin)
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
find_package(rviz_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)
set(CMAKE_AUTOMOC ON)
ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED
            src/NeoFleetRViz2Plugin.cpp include/NeoFleetRViz2Plugin.hpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  rviz_common
  nav2_msgs
  geometry_msgs
  rviz_rendering
  rviz_default_plugins
  std_msgs
  )

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Link ament packages
ament_target_dependencies(${PROJECT_NAME} rclcpp rviz_common geometry_msgs nav2_msgs)

# Link non ament packages
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


install(DIRECTORY include
   DESTINATION include
)

install(FILES 
  plugins_description.xml
  DESTINATION share
  )

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
)

install(TARGETS ${PROJECT_NAME}
TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}
DESTINATION lib
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
INCLUDES DESTINATION include
  )

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_package()