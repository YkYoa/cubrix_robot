cmake_minimum_required(VERSION 3.8)
project(ar_projects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(include)

# Motion types library
add_library(motion_types SHARED
  src/motion_types/base_motion.cpp
  src/motion_types/move_joint.cpp
  src/motion_types/delay_motion.cpp
  src/motion_types/set_planner.cpp
  src/motion_types/set_velocity.cpp
)
target_include_directories(motion_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motion_types yaml-cpp)
ament_target_dependencies(motion_types
  rclcpp
)

# YAML parser and XML/YAML generator library
add_library(project_generator SHARED
  src/yaml_parser.cpp
  src/xml_generator.cpp
  src/yaml_generator.cpp
  src/project_manager.cpp
)
target_include_directories(project_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(project_generator motion_types yaml-cpp)
ament_target_dependencies(project_generator
  rclcpp
  ament_index_cpp
)

# Project runner node (combines generate + execute)
add_executable(project_runner src/project_runner_node.cpp)
target_link_libraries(project_runner project_generator)
ament_target_dependencies(project_runner
  rclcpp
  ament_index_cpp
)

# Install targets
install(TARGETS
  motion_types
  project_generator
  EXPORT export_ar_projects
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  project_runner
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install project configs
install(DIRECTORY projects/
  DESTINATION share/${PROJECT_NAME}/projects
)

# Install config
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export targets for downstream packages
ament_export_include_directories(include)
ament_export_libraries(motion_types project_generator)
ament_export_targets(export_ar_projects HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

