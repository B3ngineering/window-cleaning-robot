cmake_minimum_required(VERSION 3.0.2)
project(window_cleaner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
  MotorCommand.msg
  RobotState.msg
  CleaningResult.msg
  Stop.msg
)

add_service_files(
    FILES
    SetCleaningArea.srv
    AbortMission.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime
)