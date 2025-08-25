cmake_minimum_required(VERSION 3.8)
project(ar_computation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(SRC_FILES
  src/data_object.cpp
  src/conversion.cpp
  src/computation.cpp
)

set(DEPENDS
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${tf2_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# shared library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDS})
ament_export_dependencies(${DEPENDS})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



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

ament_package()
