cmake_minimum_required(VERSION 2.8.3)
project(dxl_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


add_service_files(
  FILES
  SetSpeed.srv
  SetPosition.srv
  GetSpeed.srv
  GetPosition.srv
  Ping.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
