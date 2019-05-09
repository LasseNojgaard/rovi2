cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_driver)

## Uncomment if the package has a setup.py

find_package(catkin REQUIRED COMPONENTS roslaunch rospy)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/robotiq_2f_driver bin/robotiq_2f_test 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
