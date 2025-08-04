cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(joint_link_def_test
  joint_link_def_test.cpp
  TIMEOUT 30
)



if(TARGET joint_link_def_test)
  target_link_libraries(joint_link_def_test
    ${PROJECT_NAME}
  )
  
  target_include_directories(joint_link_def_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  
  # Add any necessary dependencies
  ament_target_dependencies(joint_link_def_test
    ament_cmake
  )
endif()



# Add planner definition test
ament_add_gtest(planner_def_test
  planner_def_test.cpp
  TIMEOUT 30
)

if(TARGET planner_def_test)
  target_link_libraries(planner_def_test
    ${PROJECT_NAME}
  )
  
  target_include_directories(planner_def_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
endif()