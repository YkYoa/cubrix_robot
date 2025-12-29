cmake_minimum_required(VERSION 3.8)
project(ar_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ar_planning_interface REQUIRED)
find_package(ar_motion_planner REQUIRED)
find_package(ar_scene REQUIRED)
find_package(ar_common REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# BT Action Nodes Library
add_library(ar_bt_nodes SHARED
  src/bt_action_nodes.cpp
  src/bt_condition_nodes.cpp
)

target_include_directories(ar_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ar_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  ar_planning_interface
  ar_scene
  ar_common
)

# BT Executor Node (one-shot execution)
add_executable(bt_executor_node
  src/bt_executor_node.cpp
)

target_include_directories(bt_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_executor_node
  rclcpp
  behaviortree_cpp_v3
  ar_planning_interface
)

target_link_libraries(bt_executor_node
  ar_bt_nodes
)

# BT Server Node (continuous, accepts commands from RViz UI)
add_executable(bt_server_node
  src/bt_server_node.cpp
)

target_include_directories(bt_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_server_node
  rclcpp
  behaviortree_cpp_v3
  ar_planning_interface
  std_msgs
)

target_link_libraries(bt_server_node
  ar_bt_nodes
)

# Install
install(TARGETS ar_bt_nodes
  EXPORT ar_bt_nodes_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_executor_node bt_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export
ament_export_targets(ar_bt_nodes_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  ar_planning_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
