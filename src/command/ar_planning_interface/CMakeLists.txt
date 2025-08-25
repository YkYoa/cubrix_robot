cmake_minimum_required(VERSION 3.8)
project(ar_planning_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ar_common REQUIRED)

set(SRC_FILES
  src/planning_base.cpp
  src/planning_interface.cpp
)

include_directories(
  include
  ${ar_common_INCLUDE_DIRS}
)

add_library(ar_planning_interface
  src/planning_base.cpp
  src/ar_planning_interface.cpp
)

target_link_libraries(ar_planning_interface
  ${rclcpp_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_visual_tools_LIBRARIES}
  ${moveit_msgs_LIBRARIES}
  ${geometric_shapes_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
  ${ar_common_LIBRARIES}
)

install(TARGETS ar_planning_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/ar_planning_interface/
  DESTINATION include
)

set(DEPENDS
  ament_cmake
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs
  geometric_shapes
  tf2_eigen
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   add_executable(ar_planning_interface_test test/ar_planning_interface_test.cpp)
#   ament_target_dependencies(ar_planning_interface_test rclcpp moveit_core moveit_msgs moveit_ros_planning_interface geometric_shapes tf2_eigen ar_common)
#   target_link_libraries(ar_planning_interface_test ar_planning_interface)

#   install(TARGETS ar_planning_interface_test
#     DESTINATION lib/${PROJECT_NAME}
#   )

#   ament_auto_add_test(ar_planning_interface_test ar_planning_interface_test)
# endif()





ament_package()
