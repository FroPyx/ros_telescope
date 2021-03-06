cmake_minimum_required(VERSION 3.0.2)
project(mount_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp serial std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_mount_interface
  src/MountInterface.cpp
)
add_dependencies(${PROJECT_NAME}_mount_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mount_interface ${catkin_LIBRARIES})

add_executable(mount_interface_node src/mount_interface_node.cpp)
add_dependencies(mount_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mount_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME}_mount_interface)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
