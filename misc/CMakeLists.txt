cmake_minimum_required(VERSION 2.8.3)
project(misc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#add_library(${PROJECT_NAME}_global_plan_publisher
#  src/global_plan_publisher/GlobalPlanPublisher.cpp
#)
#add_dependencies(${PROJECT_NAME}_global_plan_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_global_plan_publisher
#  ${catkin_LIBRARIES}
#)


#add_dependencies(${PROJECT_NAME}_status_handler ${PROJECT_NAME}_publish_global_plan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(global_plan_publisher_node src/global_plan_publisher/global_plan_publisher_node.cpp)
#add_dependencies(global_plan_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(global_plan_publisher_node
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}_global_plan_publisher
#)
