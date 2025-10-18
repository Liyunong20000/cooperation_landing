cmake_minimum_required(VERSION 3.0.2)
project(cooperation_landing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
  message_generation
  geometry_msgs
  jsk_network_tools
  nav_msgs
  apriltag_ros
  aerial_robot_msgs
  spinal
)

add_message_files(
  FILES
  UAV2GR.msg
  GR2UAV.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
   geometry_msgs
   apriltag_ros
   aerial_robot_msgs
   apriltag_ros
   spinal
   )

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
