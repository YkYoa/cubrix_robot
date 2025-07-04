cmake_minimum_required(VERSION 3.5)
project(devices)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ar_msg REQUIRED)
find_package(ar_serial REQUIRED)

add_library(devices_server
  src/devices_server.cpp
)
target_include_directories(devices_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(devices_server rclcpp ar_msg ar_serial)

add_executable(uart_server
  src/uart_server.cpp
)
target_include_directories(uart_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(uart_server rclcpp ar_msg ar_serial)

install(TARGETS devices_server uart_server
  EXPORT export_devices
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_devices HAS_LIBRARY_TARGET)
ament_package()