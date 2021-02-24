cmake_minimum_required(VERSION 3.0.2)
project(mount_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/MountInterface.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
