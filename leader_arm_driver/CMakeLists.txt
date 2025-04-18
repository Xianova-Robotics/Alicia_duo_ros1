cmake_minimum_required(VERSION 3.0.2)
project(leader_arm_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  frame_msgs
  message_generation  # Add message_generation for custom messages
  serial_server_node  # Add the dependency for serial_server_node
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  serial_server_node  # Ensure serial_server_node is included for message generation
)

## Declare the package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs frame_msgs message_runtime serial_server_node  # message_runtime for custom message generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# =====================
# Joint Publisher Node
# =====================
add_executable(leader_joint_publisher_node src/leader_joint_publisher.cpp)
target_link_libraries(leader_joint_publisher_node ${catkin_LIBRARIES})
add_dependencies(leader_joint_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =====================
# Joint Listener Node
# =====================
add_executable(leader_joint_listener_node src/leader_joint_listener.cpp)
target_link_libraries(leader_joint_listener_node ${catkin_LIBRARIES})
add_dependencies(leader_joint_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =====================
# Serial Communication Helper Library
# =====================
add_library(${PROJECT_NAME}_serial_comm_helper
  src/serial_comm_helper.cpp
)
add_dependencies(${PROJECT_NAME}_serial_comm_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_serial_comm_helper ${catkin_LIBRARIES})

# =====================
# Test Serial Read Node
# =====================
add_executable(test_serial_read_node src/test_serial_read_main.cpp)
add_dependencies(test_serial_read_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_serial_read_node
  ${PROJECT_NAME}_serial_comm_helper
  ${catkin_LIBRARIES}
)

# =====================
# Alicia Hardware Interface Library
# =====================
add_library(alicia_hw
  src/alicia_hw.cpp
)
add_dependencies(alicia_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(alicia_hw ${catkin_LIBRARIES})

# =====================
# Hardware Interface Node (Main Entry Point)
# =====================
add_executable(hardware_interface_node src/hardware_interface_node.cpp)
target_link_libraries(hardware_interface_node
  alicia_hw
  ${PROJECT_NAME}_serial_comm_helper     
  ${catkin_LIBRARIES}
)
add_dependencies(hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
