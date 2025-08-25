cmake_minimum_required(VERSION 3.8)
project(ar_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)

# Only needed if you use pluginlib in your .cpp
# find_package(pluginlib REQUIRED)

# include directories
include_directories(
  include
)


# build library
add_library(ar_motion_planner
  src/ar_motion_planner.cpp
)

ament_target_dependencies(ar_motion_planner
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometric_shapes
  tf2_eigen
)


# install library
install(TARGETS ar_motion_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install headers
install(DIRECTORY
  include/ar_motion_planner/
  DESTINATION include
)

# install config
install(FILES
  config/ar_motion_planner.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# --- tests ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(ar_motion_planner_test test/ar_motion_planner_test.cpp)
  if(TARGET ar_motion_planner_test)
    target_link_libraries(ar_motion_planner_test ar_motion_planner)
    ament_target_dependencies(ar_motion_planner_test
      rclcpp moveit_core moveit_msgs geometric_shapes tf2_eigen
    )
  endif()
endif()

ament_package()
