cmake_minimum_required(VERSION 2.8.3)
project(mount_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(DIRECTORY msg
  FILES
  EquatorialCoordinates.msg
  EquatorialCoordinatesText.msg
)

add_service_files(DIRECTORY srv
  FILES
)

#add_action_files(DIRECTORY action
#  FILES
#)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

