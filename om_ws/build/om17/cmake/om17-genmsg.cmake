# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "om17: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iom17:/home/phobos/om_ws/src/om17/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iom17:/home/phobos/om_ws/src/om17/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(om17_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_custom_target(_om17_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "om17" "/home/phobos/om_ws/src/om17/msg/CellCost.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(om17
  "/home/phobos/om_ws/src/om17/msg/CellCost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/om17
)

### Generating Services

### Generating Module File
_generate_module_cpp(om17
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/om17
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(om17_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(om17_generate_messages om17_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_dependencies(om17_generate_messages_cpp _om17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(om17_gencpp)
add_dependencies(om17_gencpp om17_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS om17_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(om17
  "/home/phobos/om_ws/src/om17/msg/CellCost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/om17
)

### Generating Services

### Generating Module File
_generate_module_eus(om17
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/om17
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(om17_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(om17_generate_messages om17_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_dependencies(om17_generate_messages_eus _om17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(om17_geneus)
add_dependencies(om17_geneus om17_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS om17_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(om17
  "/home/phobos/om_ws/src/om17/msg/CellCost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/om17
)

### Generating Services

### Generating Module File
_generate_module_lisp(om17
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/om17
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(om17_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(om17_generate_messages om17_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_dependencies(om17_generate_messages_lisp _om17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(om17_genlisp)
add_dependencies(om17_genlisp om17_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS om17_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(om17
  "/home/phobos/om_ws/src/om17/msg/CellCost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/om17
)

### Generating Services

### Generating Module File
_generate_module_nodejs(om17
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/om17
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(om17_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(om17_generate_messages om17_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_dependencies(om17_generate_messages_nodejs _om17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(om17_gennodejs)
add_dependencies(om17_gennodejs om17_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS om17_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(om17
  "/home/phobos/om_ws/src/om17/msg/CellCost.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/om17
)

### Generating Services

### Generating Module File
_generate_module_py(om17
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/om17
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(om17_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(om17_generate_messages om17_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phobos/om_ws/src/om17/msg/CellCost.msg" NAME_WE)
add_dependencies(om17_generate_messages_py _om17_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(om17_genpy)
add_dependencies(om17_genpy om17_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS om17_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/om17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/om17
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(om17_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET om17_generate_messages_cpp)
  add_dependencies(om17_generate_messages_cpp om17_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/om17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/om17
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(om17_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET om17_generate_messages_eus)
  add_dependencies(om17_generate_messages_eus om17_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/om17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/om17
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(om17_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET om17_generate_messages_lisp)
  add_dependencies(om17_generate_messages_lisp om17_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/om17)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/om17
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(om17_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET om17_generate_messages_nodejs)
  add_dependencies(om17_generate_messages_nodejs om17_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/om17)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/om17\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/om17
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(om17_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET om17_generate_messages_py)
  add_dependencies(om17_generate_messages_py om17_generate_messages_py)
endif()
