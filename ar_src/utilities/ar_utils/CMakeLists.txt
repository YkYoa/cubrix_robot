cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME ar_utils)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTING "Enable building of tests" OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)  
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)

set(SRC_FILES
    src/ar_utils.cpp
    src/ar_logging.cpp
    src/ar_serializer.cpp
)

set(DEPENDS
    ament_cmake
    yaml_cpp_vendor
    yaml-cpp
    rclcpp
    rclcpp_lifecycle
    Boost
    Eigen3
    moveit_core
)

include_directories(${PROJECT_NAME}
    PUBLIC
    include
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS})

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${DEPENDS}
)
ament_export_dependencies(${DEPENDS})

# Install libraries and headers
install(DIRECTORY include/
        DESTINATION include)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

# Test handling - FIXED SECTION
if(BUILD_TESTING)
    message(STATUS "Building tests...")
    
    # Find testing dependencies
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    
    # Build test_log executable
    add_executable(test_log test/test_log.cpp)
    target_link_libraries(test_log ${PROJECT_NAME})
    ament_target_dependencies(test_log ${DEPENDS})
    
    # Add as test (instead of installing)
    ament_add_test(test_log
        COMMAND "$<TARGET_FILE:test_log>"
        TIMEOUT 30
    )
    
    # Optionally add linting
    ament_lint_auto_find_test_dependencies()
endif()

# Package the library
ament_package()