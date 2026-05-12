cmake_minimum_required(VERSION 3.8)

project(simple_message)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(industrial_msgs REQUIRED)

# The simple message make file builds three libraries: simple_message,
# simple_message_bswap and simple_message_float64.
#
# simple_message - is the default library.  This library should be used
# when the target for the simple message is the same endian (i.e. both
# big-endian or little-endian).  Intel based machines are little endian
#
# simple_message_bswap - is an alternative library that can be used
# when the target for simple message is a DIFFERENT endian AND when the target
# target cannot perform byte swapping (as is the case for some industrial
# controllers).  This library performs byte swapping at the lowest load/unload
# levels.
#
# simple_message_float64 - another alternative which uses 8 byte floats instead
# of 4 byte floats (ie: double precision vs single precision).
# This variant changes the shared_float typedef everywhere to be 8 byte floats.
#
# NOTE: The libraries generated this package are not included in the catkin_package
# macro because libraries must be explicitly linked in projects that depend on this
# package.  If this is not done (and these libraries were exported), then multiple
# library definitions (normal - simple_message, byteswapped - simple_message_bswap
# and simple_message_float64) are all included (this is bad).

include_directories(include)

# Build static libs, to reduce dependency-chaining for industrial_robot_client
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)

#catkin_lint: ignore_once literal_project_name
set(SRC_FILES src/byte_array.cpp
	src/simple_message.cpp
	src/smpl_msg_connection.cpp

	src/socket/simple_socket.cpp
	src/socket/tcp_client.cpp
	src/socket/tcp_server.cpp
	src/socket/tcp_socket.cpp
	src/socket/udp_client.cpp
	src/socket/udp_server.cpp
	src/socket/udp_socket.cpp

	src/joint_data.cpp
	src/joint_feedback.cpp
	src/joint_traj_pt_full.cpp
	src/joint_traj_pt.cpp
	src/joint_traj.cpp
	src/message_handler.cpp
	src/message_manager.cpp
	src/ping_handler.cpp
	src/ping_message.cpp
	src/robot_status.cpp

	src/messages/joint_feedback_message.cpp
	src/messages/joint_message.cpp
	src/messages/joint_traj_pt_full_message.cpp
	src/messages/joint_traj_pt_message.cpp
	src/messages/robot_status_message.cpp

	src/simple_comms_fault_handler.cpp
)

set(UTEST_SRC_FILES test/utest.cpp test/utest_message.cpp)

# DEFAULT LIBRARY (SAME ENDIAN)
#catkin_lint: ignore_once unsorted_list
add_library(${PROJECT_NAME} ${SRC_FILES})
# NOTE: keep these in-sync with the lists in 'cmake/platform_build_flags.cmake'.
target_compile_definitions(${PROJECT_NAME} PUBLIC
  SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX)
ament_target_dependencies(${PROJECT_NAME} rclcpp industrial_msgs)
target_link_libraries(${PROJECT_NAME} $<$<BOOL:${WIN32}>:Ws2_32.lib>)

# ALTERNATIVE LIBRARY (DIFFERENT ENDIAN)
#catkin_lint: ignore_once unsorted_list
add_library(${PROJECT_NAME}_bswap ${SRC_FILES})
# NOTE: keep these in-sync with the lists in 'cmake/platform_build_flags.cmake'
target_compile_definitions(${PROJECT_NAME}_bswap PUBLIC
  SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX BYTE_SWAPPING)
ament_target_dependencies(${PROJECT_NAME}_bswap rclcpp industrial_msgs)
target_link_libraries(${PROJECT_NAME}_bswap $<$<BOOL:${WIN32}>:Ws2_32.lib>)

# ALTERNATIVE LIBRARY (64-bit floats)
#catkin_lint: ignore_once unsorted_list
add_library(${PROJECT_NAME}_float64 ${SRC_FILES})
# NOTE: keep these in-sync with the lists in 'cmake/platform_build_flags.cmake'
target_compile_definitions(${PROJECT_NAME}_float64 PUBLIC
  SIMPLE_MESSAGE_USE_ROS SIMPLE_MESSAGE_LINUX FLOAT64)
ament_target_dependencies(${PROJECT_NAME}_float64 rclcpp industrial_msgs)
target_link_libraries(${PROJECT_NAME}_float64 $<$<BOOL:${WIN32}>:Ws2_32.lib>)


# NOTE: All test files require TEST_PORT_BASE to be defined.  Defining different
# ports for each test executable allows them to run in parallel.
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(utest ${UTEST_SRC_FILES})
    target_compile_definitions(utest PRIVATE TEST_PORT_BASE=11000)
    target_link_libraries(utest ${PROJECT_NAME})

    ament_add_gtest(utest_byte_swapping ${UTEST_SRC_FILES})
    target_compile_definitions(utest_byte_swapping PRIVATE TEST_PORT_BASE=12000)
    target_link_libraries(utest_byte_swapping ${PROJECT_NAME}_bswap)

    ament_add_gtest(utest_float64 ${UTEST_SRC_FILES})
    target_compile_definitions(utest_float64 PRIVATE TEST_PORT_BASE=13000 FLOAT64)
    target_link_libraries(utest_float64 ${PROJECT_NAME}_float64)

    ament_add_gtest(utest_udp ${UTEST_SRC_FILES})
    target_compile_definitions(utest_udp PRIVATE TEST_PORT_BASE=15000 UDP_TEST)
    target_link_libraries(utest_udp ${PROJECT_NAME})
endif()

install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bswap ${PROJECT_NAME}_float64
    EXPORT simple_messageTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(simple_messageTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp industrial_msgs)

ament_package()
