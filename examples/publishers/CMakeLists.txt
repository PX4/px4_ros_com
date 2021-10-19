cmake_minimum_required(VERSION 3.5)
project(px4_ros_com_publisher_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(debug_vect_publisher src/01_debug_vect_publisher.cpp)
ament_target_dependencies(debug_vect_publisher rclcpp px4_msgs)

add_executable(arm_publisher src/02_arm_publisher.cpp)
ament_target_dependencies(arm_publisher rclcpp px4_msgs)

add_executable(disarm_publisher src/03_disarm_publisher.cpp)
ament_target_dependencies(disarm_publisher rclcpp px4_msgs)

add_executable(takeoff_publisher src/04_takeoff_publisher.cpp)
ament_target_dependencies(takeoff_publisher rclcpp px4_msgs)

add_executable(land_publisher src/05_land_publisher.cpp)
ament_target_dependencies(land_publisher rclcpp px4_msgs)

ament_package()

install(
    TARGETS
        debug_vect_publisher
        arm_publisher
        disarm_publisher
        takeoff_publisher
        land_publisher
    DESTINATION 
        lib/${PROJECT_NAME})
