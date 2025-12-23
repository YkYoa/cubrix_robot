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
  src/planning_interface.cpp
)

include_directories(
  include
  ${ar_common_INCLUDE_DIRS}
)

add_library(ar_planning_interface
  src/ar_planning_interface.cpp
)

target_include_directories(ar_planning_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ar_planning_interface
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs
  geometric_shapes
  tf2_eigen
  ar_common
)

install(TARGETS ar_planning_interface
  EXPORT ar_planning_interface_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

# install test configs and launch files
install(DIRECTORY
  test/config/
  DESTINATION share/${PROJECT_NAME}/test/config
)

install(DIRECTORY
  test/launch/
  DESTINATION share/${PROJECT_NAME}/test/launch
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

# Build test executable
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Build interactive test executable
  add_executable(ar_planning_interface_test_node test/ar_planning_interface_test.cpp)
  
  target_include_directories(ar_planning_interface_test_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(ar_planning_interface_test_node 
    rclcpp 
    moveit_core 
    moveit_msgs 
    moveit_ros_planning_interface 
    geometric_shapes 
    tf2_eigen 
    ar_common
  )
  
  target_link_libraries(ar_planning_interface_test_node ar_planning_interface)

  install(TARGETS ar_planning_interface_test_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()




ament_export_targets(ar_planning_interface_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  moveit_msgs
  geometric_shapes
  tf2_eigen
)

ament_package()
