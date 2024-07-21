cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
find_package(perception_3d REQUIRED)
find_package(dddmr_sys_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_eigen
  pluginlib
  pcl_conversions
  perception_3d
  dddmr_sys_core
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# a_star_on_graph
add_library(a_star_on_graph SHARED
  src/a_star_on_graph.cpp
)
target_link_libraries(a_star_on_graph
  ${PCL_LIBRARIES}
)
ament_target_dependencies(a_star_on_graph
  ${dependencies}
)

# global_planner
add_library(global_planner SHARED
  src/global_planner.cpp
)
target_link_libraries(global_planner
  a_star_on_graph
)
ament_target_dependencies(global_planner
  ${dependencies}
)

#global_planner_node
add_executable(global_planner_node
  src/global_planner_node.cpp
)
target_link_libraries(global_planner_node 
  global_planner
)
ament_target_dependencies(global_planner_node
  ${dependencies}
)


install(TARGETS
  a_star_on_graph
  global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#install node
install(TARGETS
  global_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  data
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  script/example_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()