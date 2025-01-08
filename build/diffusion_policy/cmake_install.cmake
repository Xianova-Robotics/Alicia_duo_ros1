# Install script for directory: /home/eto/Alicia_duo_ros/src/diffusion_policy

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/eto/Alicia_duo_ros/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diffusion_policy/msg" TYPE FILE FILES
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_angles_list.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/get_velocity_list.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose_vel.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/eef_pose.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_angles.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/joint_vel.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/obsdata.msg"
    "/home/eto/Alicia_duo_ros/src/diffusion_policy/msg/cameradata.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diffusion_policy/cmake" TYPE FILE FILES "/home/eto/Alicia_duo_ros/build/diffusion_policy/catkin_generated/installspace/diffusion_policy-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/eto/Alicia_duo_ros/devel/include/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/eto/Alicia_duo_ros/devel/share/roseus/ros/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/eto/Alicia_duo_ros/devel/share/common-lisp/ros/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/eto/Alicia_duo_ros/devel/share/gennodejs/ros/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/eto/Alicia_duo_ros/devel/lib/python3/dist-packages/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/eto/Alicia_duo_ros/devel/lib/python3/dist-packages/diffusion_policy")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/eto/Alicia_duo_ros/build/diffusion_policy/catkin_generated/installspace/diffusion_policy.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diffusion_policy/cmake" TYPE FILE FILES "/home/eto/Alicia_duo_ros/build/diffusion_policy/catkin_generated/installspace/diffusion_policy-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diffusion_policy/cmake" TYPE FILE FILES
    "/home/eto/Alicia_duo_ros/build/diffusion_policy/catkin_generated/installspace/diffusion_policyConfig.cmake"
    "/home/eto/Alicia_duo_ros/build/diffusion_policy/catkin_generated/installspace/diffusion_policyConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diffusion_policy" TYPE FILE FILES "/home/eto/Alicia_duo_ros/src/diffusion_policy/package.xml")
endif()

