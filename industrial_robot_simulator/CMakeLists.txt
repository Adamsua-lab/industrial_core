cmake_minimum_required(VERSION 3.8)
project(industrial_robot_simulator)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  industrial_robot_simulator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
