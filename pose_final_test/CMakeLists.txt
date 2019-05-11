cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pose_final_test)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pose_final_test pose_final_test.cpp)
target_link_libraries(pose_final_test ${PCL_LIBRARIES})

