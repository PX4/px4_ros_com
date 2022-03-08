cmake_minimum_required(VERSION 3.5)
project(px4_ros_com_listener_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(sensor_combined_listener src/01_sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)

add_executable(vehicle_odometry_listener src/02_vehicle_odometry_listener.cpp)
ament_target_dependencies(vehicle_odometry_listener rclcpp px4_msgs)

ament_package()

install(
    TARGETS
        sensor_combined_listener
        vehicle_odometry_listener
    DESTINATION 
        lib/${PROJECT_NAME})
