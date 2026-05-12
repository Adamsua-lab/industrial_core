cmake_minimum_required(VERSION 3.8)

project(industrial_robot_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(industrial_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_message REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

set(SRC_FILES src/joint_relay_handler.cpp
              src/joint_trajectory_downloader.cpp
              src/joint_trajectory_interface.cpp
              src/joint_trajectory_streamer.cpp
              src/robot_state_interface.cpp
              src/robot_status_relay_handler.cpp
              src/utils.cpp)

include_directories(include)

# This library depends on the simple_message library, which is available
# in two formats to support endian differences between robot & PC.
#
# As in simple_message, two libraries are created: one that does direct
# passthrough of binary numeric data to the robot controller, and one that
# uses byte-swapping to account for endian mis-matches.
#
# Due to ament dependency limitations, higher-level code must
# explicitly link to the desired industrial_robot_client library:
#    target_link_libraries(my_node industrial_robot_client)
#      or
#    target_link_libraries(my_node industrial_robot_client_bswap)
#

add_library(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_action control_msgs industrial_msgs industrial_utils sensor_msgs std_msgs trajectory_msgs urdf)
target_link_libraries(${PROJECT_NAME} simple_message)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  ${simple_message_DEFINITIONS})

add_library(${PROJECT_NAME}_bswap ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME}_bswap rclcpp rclcpp_action control_msgs industrial_msgs industrial_utils sensor_msgs std_msgs trajectory_msgs urdf)
target_link_libraries(${PROJECT_NAME}_bswap simple_message_bswap)
target_compile_definitions(${PROJECT_NAME}_bswap PUBLIC
  ${simple_message_bswap_DEFINITIONS})

# The following executables(nodes) are for applications where the robot
# controller and pc have the same byte order (i.e. byte swapping NOT
# required)

add_executable(robot_state
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state
  ${PROJECT_NAME}
  simple_message)
ament_target_dependencies(robot_state rclcpp)
target_compile_definitions(robot_state PRIVATE
  ${simple_message_DEFINITIONS})

add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface
  ${PROJECT_NAME}
  simple_message)
ament_target_dependencies(motion_streaming_interface rclcpp)
target_compile_definitions(motion_streaming_interface PRIVATE
  ${simple_message_DEFINITIONS})

add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface
  ${PROJECT_NAME}
  simple_message)
ament_target_dependencies(motion_download_interface rclcpp)
target_compile_definitions(motion_download_interface PRIVATE
  ${simple_message_DEFINITIONS})

# The following executables(nodes) are for applications where the robot
# controller and pc have different same byte order (i.e. byte swapping IS
# required)

add_executable(robot_state_bswap
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state_bswap
  ${PROJECT_NAME}_bswap
  simple_message_bswap)
ament_target_dependencies(robot_state_bswap rclcpp)
target_compile_definitions(robot_state_bswap PRIVATE
  ${simple_message_bswap_DEFINITIONS})

add_executable(motion_streaming_interface_bswap
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface_bswap
  ${PROJECT_NAME}_bswap
  simple_message_bswap)
ament_target_dependencies(motion_streaming_interface_bswap rclcpp)
target_compile_definitions(motion_streaming_interface_bswap PRIVATE
  ${simple_message_bswap_DEFINITIONS})

add_executable(motion_download_interface_bswap
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface_bswap
  ${PROJECT_NAME}_bswap
  simple_message_bswap)
ament_target_dependencies(motion_download_interface_bswap rclcpp)
target_compile_definitions(motion_download_interface_bswap PRIVATE
  ${simple_message_bswap_DEFINITIONS})

add_executable(joint_trajectory_action
  src/generic_joint_trajectory_action_node.cpp
  src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action PRIVATE
  ${PROJECT_NAME})
ament_target_dependencies(joint_trajectory_action rclcpp rclcpp_action)

##########
## Test ##
##########
# Testing - Only performed on normal (non byte swapped library)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(utest_robot_client test/utest.cpp)
  ament_target_dependencies(utest_robot_client rclcpp)
  target_link_libraries(utest_robot_client
    ${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bswap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS
    joint_trajectory_action
    motion_download_interface
    motion_download_interface_bswap
    motion_streaming_interface
    motion_streaming_interface_bswap
    robot_state
    robot_state_bswap
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS
    cmake/issue46_workaround.cmake
)
