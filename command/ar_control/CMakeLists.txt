cmake_minimum_required(VERSION 3.22)
project(ar_control LANGUAGES CXX)


if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_planners REQUIRED)
find_package(ar_utils REQUIRED)
find_package(ar_action REQUIRED)
find_package(GSL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)



set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake

    moveit_common
    moveit_core
    moveit_msgs
    moveit_ros_move_group
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_planners

    hardware_interface
    pluginlib
    rclcpp
    rclcpp_action
    rclcpp_components
    rclcpp_lifecycle

    yaml_cpp_vendor
    urdf
    GSL
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ${ament_cmake_INCLUDE_DIRS}
    ${devices_INCLUDE_DIRS}
    ${utils_INCLUDE_DIRS}
    ${master_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIRS}
)

include_directories(
    include
    ${THIS_PACKAGE_INCLUDE_DIRS}
)

file(GLOB CONTROL_SOURCES
    src/ar_system.cpp
    src/ar_control_server.cpp
    src/ar_control_client.cpp
    src/ar_hardware_interface.cpp
)

add_library(
    ${PROJECT_NAME}
    SHARED
    ${CONTROL_SOURCES}
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

add_executable(ar_control_server
    ${CONTROL_SOURCES}
)
target_include_directories(ar_control_server PRIVATE
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
)
ament_target_dependencies(ar_control_server
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


add_executable(ar_control_client
    ${CONTROL_SOURCES}
)
target_include_directories(ar_control_client PRIVATE
  include
  ${THIS_PACKAGE_INCLUDE_DIRS}
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

# INSTALL
install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install (TARGETS
  ar_control_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install tomo_control_client
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



