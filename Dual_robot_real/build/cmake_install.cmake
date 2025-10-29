# Install script for directory: /media/jt/Extreme SSD/Github/Dual_robot_real/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/jt/Extreme SSD/Github/Dual_robot_real/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/setup.bash;/media/jt/Extreme SSD/Github/Dual_robot_real/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE FILE FILES
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/setup.bash"
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/setup.sh;/media/jt/Extreme SSD/Github/Dual_robot_real/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE FILE FILES
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/setup.sh"
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/setup.zsh;/media/jt/Extreme SSD/Github/Dual_robot_real/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE FILE FILES
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/setup.zsh"
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/media/jt/Extreme SSD/Github/Dual_robot_real/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/media/jt/Extreme SSD/Github/Dual_robot_real/install" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/gtest/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_ur5e_driver/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_driver/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_gazebo/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_gripper/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_simulation/robotiq_85_simulation/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_3f_gripper_articulated_msgs/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/ur_pykdl/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_gazebo/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/move_demo/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/real_robot/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_ethercat/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_2f_gripper_control/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_ft_sensor/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_modbus_rtu/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_modbus_tcp/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_bringup/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_description/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_msgs/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/si_utils/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/general-message-pkgs/path_navigation_msgs/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_2f_gripper_action_server/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_3f_gripper_control/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/general-message-pkgs/object_msgs/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_3f_gripper_joint_state_publisher/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo_ros_link_attacher/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_test_tools/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_version_helpers/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_grasp_plugin/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_grasp_plugin_ros/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_world_plugin_loader/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/livox_laser_simulation/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/general-message-pkgs/object_msgs_tools/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/gazebo-pkgs/gazebo_state_plugins/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_simulation/roboticsgroup_gazebo_plugins/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/realsense_ros_gazebo/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_3f_gripper_visualization/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_3f_rviz/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/visual_realsense/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_description/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/robotiq_85_gripper/robotiq_85_moveit_config/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/lib/ur_kinematics/cmake_install.cmake")
  include("/media/jt/Extreme SSD/Github/Dual_robot_real/build/ur_robot_driver/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/jt/Extreme SSD/Github/Dual_robot_real/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
