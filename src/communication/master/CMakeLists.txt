cmake_minimum_required(VERSION 3.8)
project(master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -w -Wall -Wextra -Wpedantic -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -lpthread -lrt)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(ament_cmake REQUIRED)
find_package(soem REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ar_utils)
find_package(ar_common REQUIRED)
find_package(Threads REQUIRED)

find_library(ETHERCAT_LIBRARY ethercat PATHS /opt/etherlab/lib /usr/local/lib REQUIRED)
include_directories(/opt/etherlab/include /usr/local/include)

ament_export_include_directories(
  include
  ${soem_INCLUDE_DIRS}/soem
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ethercat_manager.cpp
  src/soem_cpp.cpp
  src/igh_manager.cpp
  src/igh_slave.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  include/ar_ethercat/
  ${soem_INCLUDE_DIRS}/soem
  ${Boost_INCLUDE_DIRS}
  ar_utils
  ETHERCAT_LIBRARY
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${ETHERCAT_LIBRARY})

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  yaml_cpp_vendor
  soem
  ar_utils
  ar_common
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
