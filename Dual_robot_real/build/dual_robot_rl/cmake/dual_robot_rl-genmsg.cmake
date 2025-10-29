# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "dual_robot_rl: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idual_robot_rl:/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(dual_robot_rl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_custom_target(_dual_robot_rl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "dual_robot_rl" "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(dual_robot_rl
  "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dual_robot_rl
)

### Generating Services

### Generating Module File
_generate_module_cpp(dual_robot_rl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dual_robot_rl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(dual_robot_rl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(dual_robot_rl_generate_messages dual_robot_rl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_dependencies(dual_robot_rl_generate_messages_cpp _dual_robot_rl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dual_robot_rl_gencpp)
add_dependencies(dual_robot_rl_gencpp dual_robot_rl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dual_robot_rl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(dual_robot_rl
  "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dual_robot_rl
)

### Generating Services

### Generating Module File
_generate_module_eus(dual_robot_rl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dual_robot_rl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(dual_robot_rl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(dual_robot_rl_generate_messages dual_robot_rl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_dependencies(dual_robot_rl_generate_messages_eus _dual_robot_rl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dual_robot_rl_geneus)
add_dependencies(dual_robot_rl_geneus dual_robot_rl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dual_robot_rl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(dual_robot_rl
  "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dual_robot_rl
)

### Generating Services

### Generating Module File
_generate_module_lisp(dual_robot_rl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dual_robot_rl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(dual_robot_rl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(dual_robot_rl_generate_messages dual_robot_rl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_dependencies(dual_robot_rl_generate_messages_lisp _dual_robot_rl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dual_robot_rl_genlisp)
add_dependencies(dual_robot_rl_genlisp dual_robot_rl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dual_robot_rl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(dual_robot_rl
  "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dual_robot_rl
)

### Generating Services

### Generating Module File
_generate_module_nodejs(dual_robot_rl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dual_robot_rl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(dual_robot_rl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(dual_robot_rl_generate_messages dual_robot_rl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_dependencies(dual_robot_rl_generate_messages_nodejs _dual_robot_rl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dual_robot_rl_gennodejs)
add_dependencies(dual_robot_rl_gennodejs dual_robot_rl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dual_robot_rl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(dual_robot_rl
  "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dual_robot_rl
)

### Generating Services

### Generating Module File
_generate_module_py(dual_robot_rl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dual_robot_rl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(dual_robot_rl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(dual_robot_rl_generate_messages dual_robot_rl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/jt/Extreme SSD/Github/Dual_robot_real/src/dual_robot_rl/msg/RLExperimentInfo.msg" NAME_WE)
add_dependencies(dual_robot_rl_generate_messages_py _dual_robot_rl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(dual_robot_rl_genpy)
add_dependencies(dual_robot_rl_genpy dual_robot_rl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS dual_robot_rl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dual_robot_rl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/dual_robot_rl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(dual_robot_rl_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dual_robot_rl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/dual_robot_rl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(dual_robot_rl_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dual_robot_rl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/dual_robot_rl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(dual_robot_rl_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dual_robot_rl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/dual_robot_rl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(dual_robot_rl_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dual_robot_rl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dual_robot_rl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/dual_robot_rl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(dual_robot_rl_generate_messages_py std_msgs_generate_messages_py)
endif()
