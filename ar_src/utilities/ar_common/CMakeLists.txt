cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME ar_common)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Single option declaration
option(BUILD_TESTING "Enable building of tests" OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)

set(SRC_FILES
    src/common.cpp
)

set(DEPENDS
    ament_cmake
)

include_directories(${PROJECT_NAME}
    PUBLIC
    include
)

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

# Single test directory inclusion
if(BUILD_TESTING)
    message(STATUS "Building tests...")
    add_subdirectory(test)
endif()


# Package the library
ament_package()