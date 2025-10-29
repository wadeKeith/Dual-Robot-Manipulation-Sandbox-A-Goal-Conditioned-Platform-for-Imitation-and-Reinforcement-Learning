cmake_minimum_required(VERSION 2.8.3)
project(real_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  # std_msgs
  # message_generation
)


# add_message_files(
#    FILES
#   #  RLExperimentInfo.msg
#  )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
  #  std_msgs
 )

catkin_package(
  DEPENDS rospy
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)