cmake_minimum_required(VERSION 3.0.2)
project(telescope_utilities)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  telescope_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs telescope_msgs
)

#add_library(${PROJECT_NAME}
#  src/StellarPositionConversion.cpp
#)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#)

install(DIRECTORY include/stellar_position_conversion
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#install (TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
