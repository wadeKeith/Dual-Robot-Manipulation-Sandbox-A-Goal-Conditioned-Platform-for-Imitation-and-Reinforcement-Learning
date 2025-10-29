# Install script for directory: /media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dual_robot_rl/msg" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dual_robot_rl/cmake" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/dual_robot_rl-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/include/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/share/roseus/ros/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/share/common-lisp/ros/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/share/gennodejs/ros/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/lib/python3/dist-packages/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/devel/lib/python3/dist-packages/dual_robot_rl")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/dual_robot_rl.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dual_robot_rl/cmake" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/dual_robot_rl-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dual_robot_rl/cmake" TYPE FILE FILES
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/dual_robot_rlConfig.cmake"
    "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/dual_robot_rlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dual_robot_rl" TYPE FILE FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dual_robot_rl" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/train.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dual_robot_rl" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/common.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dual_robot_rl" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/ur5_task_space_pick_and_place.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dual_robot_rl" TYPE PROGRAM FILES "/media/jt/Extreme SSD/Github/Dual_robot_real/build/dual_robot_rl/catkin_generated/installspace/ur5_goal_env.py")
endif()

