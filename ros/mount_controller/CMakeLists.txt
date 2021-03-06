cmake_minimum_required(VERSION 3.0.2)
project(mount_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mount_msgs
  utilities
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs mount_msgs utilities
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_mount_controller
  src/MountController.cpp
)
add_dependencies(${PROJECT_NAME}_mount_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mount_controller ${catkin_LIBRARIES})

#add_executable(mount_interface_node src/mount_interface_node.cpp)
#add_dependencies(mount_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mount_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME}_mount_interface)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
