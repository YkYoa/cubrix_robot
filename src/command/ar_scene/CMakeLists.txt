cmake_minimum_required(VERSION 3.8)
project(ar_scene)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ar_srv REQUIRED)
find_package(ar_msgs REQUIRED)
find_package(ar_common REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(ar_scene SHARED
  src/ar_mesh_server.cpp
  src/ar_scene_manager.cpp
)

target_include_directories(ar_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(ar_scene
  rclcpp
  ament_index_cpp
  moveit_ros_planning_interface
  moveit_msgs
  ar_msgs
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library
install(TARGETS ar_scene
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Mesh sequence example executable
add_executable(mesh_sequence_example src/mesh_sequence_example.cpp)
target_link_libraries(mesh_sequence_example ar_scene)
ament_target_dependencies(mesh_sequence_example
  rclcpp
  geometry_msgs
)

install(TARGETS mesh_sequence_example
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for downstream packages
ament_export_include_directories(include)
ament_export_libraries(ar_scene)
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  moveit_ros_planning_interface
  moveit_msgs
  ar_msgs
)

ament_package()
