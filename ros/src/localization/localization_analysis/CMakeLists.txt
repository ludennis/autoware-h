cmake_minimum_required(VERSION 2.8.3)
project(localization_analysis)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

catkin_install_python(
  PROGRAMS
    bin/pose_analysis_from_bag.py
    bin/pose_analysis.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
