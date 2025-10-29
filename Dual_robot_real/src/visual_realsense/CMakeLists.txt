cmake_minimum_required(VERSION 2.8.3)
project(visual_realsense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

generate_messages(
   DEPENDENCIES
  #  std_msgs
 )
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_robot
#  CATKIN_DEPENDS moveit_ros_move_group moveit_ros_planning_interface roscpp rospy std_msgs
DEPENDS rospy
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/get_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

