cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(poseEstimation)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pose_image pose_image.cpp)
target_link_libraries(pose_image ${PCL_LIBRARIES})

