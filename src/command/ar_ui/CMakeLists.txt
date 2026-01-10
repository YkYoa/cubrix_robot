cmake_minimum_required(VERSION 3.8)
project(ar_ui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ar_projects REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

# Include directories
include_directories(include)

# Qt MOC files
qt5_wrap_cpp(MOC_FILES
  include/ar_ui/behavior_tree_panel.h
  include/ar_ui/param_editor_panel.h
)

qt5_wrap_cpp(BT_MANAGER_MOC_FILES
  include/ar_ui/bt_manager_window.h
)

qt5_wrap_cpp(PARAM_EDITOR_MOC_FILES
  include/ar_ui/param_editor_window.h
)

# RViz Panel Library
add_library(${PROJECT_NAME} SHARED
  src/behavior_tree_panel.cpp
  src/param_editor_panel.cpp
  ${MOC_FILES}
)

find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  pluginlib
  std_msgs
  ar_projects
  ament_index_cpp
)

# Standalone BT Manager Application
add_executable(bt_manager
  src/bt_manager_main.cpp
  src/bt_manager_window.cpp
  ${BT_MANAGER_MOC_FILES}
)

target_include_directories(bt_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bt_manager
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

ament_target_dependencies(bt_manager
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ar_projects
)

# Standalone Parameter Editor Application
add_executable(param_editor
  src/param_editor_main.cpp
  src/param_editor_window.cpp
  ${PARAM_EDITOR_MOC_FILES}
)

target_include_directories(param_editor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(param_editor
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  yaml-cpp
)

ament_target_dependencies(param_editor
  rclcpp
  std_msgs
  ament_index_cpp
)

# Register as rviz plugin
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install targets
install(TARGETS ${PROJECT_NAME} bt_manager param_editor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

