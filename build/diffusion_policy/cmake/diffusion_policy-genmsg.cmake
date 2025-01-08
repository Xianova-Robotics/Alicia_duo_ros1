# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "diffusion_policy: 8 messages, 0 services")

set(MSG_I_FLAGS "-Idiffusion_policy:/home/eto/Alicia_duo_ros/src/diffusion_policy/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(diffusion_policy_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" ""
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_custom_target(_diffusion_policy_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diffusion_policy" "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_cpp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
)

### Generating Services

### Generating Module File
_generate_module_cpp(diffusion_policy
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(diffusion_policy_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(diffusion_policy_generate_messages diffusion_policy_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_cpp _diffusion_policy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diffusion_policy_gencpp)
add_dependencies(diffusion_policy_gencpp diffusion_policy_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diffusion_policy_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)
_generate_msg_eus(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
)

### Generating Services

### Generating Module File
_generate_module_eus(diffusion_policy
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(diffusion_policy_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(diffusion_policy_generate_messages diffusion_policy_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_eus _diffusion_policy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diffusion_policy_geneus)
add_dependencies(diffusion_policy_geneus diffusion_policy_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diffusion_policy_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)
_generate_msg_lisp(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
)

### Generating Services

### Generating Module File
_generate_module_lisp(diffusion_policy
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(diffusion_policy_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(diffusion_policy_generate_messages diffusion_policy_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_lisp _diffusion_policy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diffusion_policy_genlisp)
add_dependencies(diffusion_policy_genlisp diffusion_policy_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diffusion_policy_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)
_generate_msg_nodejs(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
)

### Generating Services

### Generating Module File
_generate_module_nodejs(diffusion_policy
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(diffusion_policy_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(diffusion_policy_generate_messages diffusion_policy_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_nodejs _diffusion_policy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diffusion_policy_gennodejs)
add_dependencies(diffusion_policy_gennodejs diffusion_policy_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diffusion_policy_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)
_generate_msg_py(diffusion_policy
  "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
)

### Generating Services

### Generating Module File
_generate_module_py(diffusion_policy
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(diffusion_policy_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(diffusion_policy_generate_messages diffusion_policy_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg" NAME_WE)
add_dependencies(diffusion_policy_generate_messages_py _diffusion_policy_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diffusion_policy_genpy)
add_dependencies(diffusion_policy_genpy diffusion_policy_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diffusion_policy_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diffusion_policy
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(diffusion_policy_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/diffusion_policy
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(diffusion_policy_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/diffusion_policy
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(diffusion_policy_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/diffusion_policy
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(diffusion_policy_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diffusion_policy
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(diffusion_policy_generate_messages_py std_msgs_generate_messages_py)
endif()
