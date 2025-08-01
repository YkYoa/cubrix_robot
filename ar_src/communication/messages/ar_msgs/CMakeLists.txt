cmake_minimum_required(VERSION 3.8)
project(ar_msgs) 

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")


rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}  # Now contains relative paths
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package()