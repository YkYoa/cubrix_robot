cmake_minimum_required(VERSION 3.22)
project(ar_control LANGUAGES CXX)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
# 	set(CMAKE_BUILD_TYPE RelWithDebInfo)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   # add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Option for simulation-only builds (UI team doesn't need EtherCAT)
option(SIMULATION_ONLY "Build without EtherCAT hardware support (for simulation)" OFF)

if(SIMULATION_ONLY)
  message(STATUS "[ar_control] Building in SIMULATION_ONLY mode - EtherCAT/IGH code excluded")
  add_definitions(-DNO_ETHERCAT)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(master REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(urdf REQUIRED)
find_package(ar_action REQUIRED)
find_package(ar_comm REQUIRED)
# find_package(ar_devices REQUIRED)
find_package(ar_utils REQUIRED)
find_package(ar_common REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(pilz_industrial_motion_planner REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_planners REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(GSL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  octomap
  octomap_msgs
  octomap_ros
  sensor_msgs
  pcl_conversions

  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  GSL

  yaml_cpp_vendor
  urdf
  urdfdom
  urdfdom_headers

  ar_action
  ar_comm
  master
  # ar_devices
  # ar_tcp
  ar_utils
  ar_common
  pilz_industrial_motion_planner
  GSL
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_planners
)

set(THIS_PACKAGE_INCLUDE_DIRS
  ${ament_cmake_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ar_action_INCLUDE_DIRS}
  ${ar_comm_INCLUDE_DIRS}
  ${master_INCLUDE_DIRS}
  # ${ar_devices_INCLUDE_DIRS}
  ${ar_utils_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIRS}
)

include_directories(
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

file(GLOB CONTORL_SOURCES
  src/ar_system.cpp
  src/ar_joint_control.cpp
  src/ar_drive_client.cpp
  src/ar_drive_control.cpp
  src/ar_hardware_interface.cpp
  src/ar_control_server.cpp
  src/ar_control_client.cpp
)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  ${CONTORL_SOURCES}
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

# ar_control_server lib
add_executable(ar_control_server
  ${CONTORL_SOURCES}
)
target_include_directories(ar_control_server PRIVATE
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(ar_control_server
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# ar_control_client lib
add_library(ar_control_client SHARED
  src/ar_control_client.cpp
)
target_include_directories(ar_control_client PRIVATE
  include
)
ament_target_dependencies(ar_control_client
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface ar_control.xml)


add_executable(ar_control_client_server_test test/ar_control_client_server_test.cpp)
ament_target_dependencies(ar_control_client_server_test
  rclcpp
  std_msgs
  yaml_cpp_vendor
  rclcpp_components
  # ar_master
  ar_utils
)
target_include_directories(ar_control_client_server_test
  PRIVATE
  include
)


# INSTALL
install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)


install (TARGETS
  ar_control_server
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install ar_control_client
install(TARGETS
  ar_control_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
	PATTERN "setup_assistant.launch" EXCLUDE)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
