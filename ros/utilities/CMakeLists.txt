cmake_minimum_required(VERSION 3.0.2)
project(utilities)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mount_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs mount_msgs
)


install(DIRECTORY include/stellar_position_conversion
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
