cmake_minimum_required(VERSION 2.8.12)
project(car_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  vision_msgs
  std_msgs
  rospy
)

add_message_files(
  FILES
  State.msg
  Reference.msg
  Trajectory.msg
  MotionPlan.msg
  MotionRequest.msg
  MotionResponse.msg
)

add_service_files(
  FILES
  planmotion.srv
  getobstacles.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
