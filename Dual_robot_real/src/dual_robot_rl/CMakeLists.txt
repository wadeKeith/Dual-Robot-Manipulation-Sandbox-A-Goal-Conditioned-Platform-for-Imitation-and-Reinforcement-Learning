cmake_minimum_required(VERSION 2.8.3)
project(dual_robot_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)


add_message_files(
   FILES
   RLExperimentInfo.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  DEPENDS message_runtime rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/train.py
  scripts/utils/common.py
  scripts/ur5_task_space_pick_and_place.py
  scripts/ur5_goal_env.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)