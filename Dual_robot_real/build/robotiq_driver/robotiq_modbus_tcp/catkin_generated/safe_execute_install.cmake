execute_process(COMMAND "/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_modbus_tcp/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/media/jt/Extreme SSD/Github/Dual_robot_real/build/robotiq_driver/robotiq_modbus_tcp/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
