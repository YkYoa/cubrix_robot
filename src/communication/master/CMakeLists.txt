cmake_minimum_required(VERSION 3.8)
project(master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Option for simulation-only builds (UI team doesn't need EtherCAT)
option(SIMULATION_ONLY "Build without EtherCAT hardware support (for simulation)" OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(NOT SIMULATION_ONLY)
    add_compile_options(-g -w -Wall -Wextra -Wpedantic -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -lpthread -lrt)
  else()
    add_compile_options(-g -w -Wall -Wextra -Wpedantic -lpthread -lrt)
  endif()
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

if(NOT SIMULATION_ONLY)
  find_library(ETHERCAT_LIBRARY ethercat PATHS /opt/etherlab/lib /usr/local/lib REQUIRED)
  include_directories(/opt/etherlab/include /usr/local/include)
  set(IGH_SOURCES
    src/igh_manager.cpp
    src/igh_slave.cpp
    src/igh_error_handler.cpp
  )
else()
  message(STATUS "Building in SIMULATION_ONLY mode - EtherCAT/IGH files excluded")
  set(ETHERCAT_LIBRARY "")
  set(IGH_SOURCES "")
  add_definitions(-DNO_ETHERCAT)
endif()

ament_export_include_directories(
  include
  ${soem_INCLUDE_DIRS}/soem
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ethercat_manager.cpp
  src/soem_cpp.cpp
  ${IGH_SOURCES}
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
