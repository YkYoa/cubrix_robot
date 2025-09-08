cmake_minimum_required(VERSION 3.8)
project(master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(soem REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ar_utils)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_include_directories(
  include
  ${soem_INCLUDE_DIRS}/soem
)

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
  ${soem_INCLUDE_DIRS}/soem
  ${Boost_INCLUDE_DIRS}
  ar_utils
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  yaml_cpp_vendor
  soem
  ar_utils
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  rclcpp
)
ament_package()
