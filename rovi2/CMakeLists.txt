cmake_minimum_required(VERSION 2.8.3)
project(rovi2)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  caros_universalrobot
  robotiq_2f_driver
  std_msgs
  wsg_50_driver
  robotiq_2f_msgs
  message_generation
  wsg_50_common
  pose_ros
  caros_sensor_msgs
  caros_teleoperation
)


## Adding message files
#add_message_files(
#   FILES
#   Grasp_cmd.msg
#    Object_pose.msg
# )
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Grasp_cmd.srv
   SBL_cmd.srv
 )
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs robotiq_2f_msgs rovi2 pose_ros)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rovi2
  CATKIN_DEPENDS 
	caros_universalrobot
	robotiq_2f_driver 
 	std_msgs 
	wsg_50_driver 
 	robotiq_2f_msgs
 	message_runtime 
	wsg_50_common 
	pose_ros 
	caros_sensor_msgs
	caros_teleoperation
	caros_common_robwork_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(              ROS_INFO("PoseA sent to SBL: ")
# include
  ${catkin_INCLUDE_DIRS}

)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rovi2_node.cpp)
add_executable(grasp src/grasp_service.cpp)
add_executable(SBL src/SBLcollision.cpp)
add_executable(rovi2Interface src/InterfaceEmil.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(grasp ${catkin_LIBRARIES})
target_link_libraries(SBL ${catkin_LIBRARIES})
target_link_libraries(rovi2Interface ${catkin_LIBRARIES})



## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(grasp rovi2_generate_messages_cpp)
add_dependencies(SBL rovi2_generate_messages_cpp)

