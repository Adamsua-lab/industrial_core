cmake_minimum_required(VERSION 3.8)

project(industrial_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

add_library(${PROJECT_NAME} src/param_utils.cpp src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp urdf)


if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(utest_inds_utils test/utest.cpp)
    ament_target_dependencies(utest_inds_utils rclcpp urdf)
    target_link_libraries(utest_inds_utils ${PROJECT_NAME})
endif()


install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME})

ament_package()
