cmake_minimum_required(VERSION 3.8)
project(ar_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch ros2_control meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

