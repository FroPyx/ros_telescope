cmake_minimum_required(VERSION 3.0.2)
project(test_unit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  telescope_utilities
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp telescope_utilities
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stellar_position_conversion_test src/stellar_position_conversion_test.cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
