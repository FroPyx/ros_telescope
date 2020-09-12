cmake_minimum_required(VERSION 2.8.3)
project(telescope_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(DIRECTORY msg
  FILES
  MotorsPositionCommand.msg
  MotorsStatus.msg
  PositionHourAngleTextFormat.msg
  SpeedCommand.msg
)

#add_service_files(DIRECTORY srv
#  FILES
#)

#add_action_files(DIRECTORY action
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

