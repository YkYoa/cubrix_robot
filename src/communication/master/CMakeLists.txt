cmake_minimum_required(VERSION 3.8)
project(master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ar_utils REQUIRED)

# ------------------------------------------------------------------------------
# SOEM (manual linking, since no CMake config is installed)
# ------------------------------------------------------------------------------
find_library(SOEM_LIB soem PATHS /usr/local/lib)
if(NOT SOEM_LIB)
  message(FATAL_ERROR "Could not find libsoem in /usr/local/lib")
endif()

include_directories(/usr/local/include)

# ------------------------------------------------------------------------------
# Library
# ------------------------------------------------------------------------------
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ethercat_manager.cpp
  src/soem_cpp.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
    include
    include/ar_ethercat/
    ${Boost_INCLUDE_DIRS}
    /usr/local/include
)

target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${SOEM_LIB}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  yaml_cpp_vendor
  ar_utils
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ------------------------------------------------------------------------------
# Exports
# ------------------------------------------------------------------------------
ament_export_include_directories(include /usr/local/include)
ament_export_libraries(${PROJECT_NAME} ${SOEM_LIB})
ament_export_dependencies(rclcpp yaml_cpp_vendor ar_utils)

ament_package()
