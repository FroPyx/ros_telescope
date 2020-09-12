cmake_minimum_required(VERSION 3.0.2)
project(telescope_motors_controler)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
