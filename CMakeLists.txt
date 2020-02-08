cmake_minimum_required(VERSION 3.5)

project(px4_ros_com)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Check if any sanitizers set
include(EnableSanitizers)

# Check if ROS_DISTRO is Dashing or Eloquent
# Required since "create_subscription()" and "create_publisher()" APIs changed
list(APPEND ROS_DISTROS "dashing" "eloquent")
if(DEFINED ENV{ROS2_DISTRO})
  if ($ENV{ROS2_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
else()
  if ($ENV{ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(rclcpp REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PythonInterp 3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

###################################
# Generate micro-RTPS agent code ##
###################################

get_filename_component(PX4_MSGS_DIR "../" ABSOLUTE BASE_DIR ${px4_msgs_DIR})
set(MSGS_DIR "${PX4_MSGS_DIR}/msg" CACHE INTERNAL "MSGS_DIR")
include(GenerateMicroRTPSAgent)

#################
# Setup targets #
#################

include_directories(include ${rclcpp_INCLUDE_DIRS} ${px4_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "px4_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Add frame_transforms lib
add_library(frame_transforms src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3)
target_link_libraries(frame_transforms Eigen3)

# Add microRTPS agent
add_executable(micrortps_agent ${MICRORTPS_AGENT_FILES})
target_link_libraries(micrortps_agent fastrtps fastcdr)

# Add listeners
custom_executable(listeners sensor_combined_listener)
custom_executable(listeners vehicle_gps_position_listener)

# Add advertiser
custom_executable(advertisers debug_vect_advertiser)

# Export information to downstream packages
ament_export_interfaces(frame_transforms HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3)
ament_export_include_directories(include ${px4_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_export_libraries()

############
# Install ##
############

# Install artifacts
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(TARGETS frame_transforms
        EXPORT frame_transforms
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(TARGETS micrortps_agent sensor_combined_listener vehicle_gps_position_listener debug_vect_advertiser
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

############
# Testing ##
############

# Install tests
install(DIRECTORY
  test
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
