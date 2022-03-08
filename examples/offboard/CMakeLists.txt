cmake_minimum_required(VERSION 3.5)
project(px4_ros_com_offboard_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)

ament_package()

install(
    TARGETS
        offboard_control
    DESTINATION 
        lib/${PROJECT_NAME})
