cmake_minimum_required(VERSION 3.8)

project(industrial_trajectory_filters)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(orocos_kdl REQUIRED)

include_directories(include
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/add_smoothing_filter.cpp
  src/n_point_filter.cpp
  src/smoothing_trajectory_filter.cpp
  src/uniform_sample_filter.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  class_loader
  moveit_core
  moveit_ros_planning
  pluginlib
  trajectory_msgs
)
target_link_libraries(${PROJECT_NAME} ${orocos_kdl_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(FILES planning_request_adapters_plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
