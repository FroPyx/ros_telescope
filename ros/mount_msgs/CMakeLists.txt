cmake_minimum_required(VERSION 2.8.3)
project(mount_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
)

add_message_files(DIRECTORY msg
  FILES
  EquatorialCoordinates.msg
  EquatorialCoordinatesText.msg
)

add_service_files(DIRECTORY srv
  FILES
)

add_action_files(DIRECTORY action
  FILES
  Guidance.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS genmsg actionlib_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

