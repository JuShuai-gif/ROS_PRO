cmake_minimum_required(VERSION 3.0.2)
project(sim_env)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  message_generation
  roscpp
  rospy
  rviz
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_env
#  CATKIN_DEPENDS controller_manager gazebo_ros message_generation roscpp rospy rviz std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
