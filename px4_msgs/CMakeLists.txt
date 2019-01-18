cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Option to set the PX4 Firmware repository directory
option(PX4_FIRMWARE_DIR "PX4 Firmware repository directory (by default not set)"
       "")
if(${PX4_FIRMWARE} NOT STREQUAL "")
  set(PX4_FIRMWARE_DIR "${PX4_FIRMWARE_DIR}" CACHE INTERNAL "PX4_FIRMWARE_DIR")
endif()

# Look for the PX4 Firmware dir and get the uORB msg definitions if
# PX4_FIRMWARE_DIR is not set
find_package(PX4Firmware REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

##############################################
# Generate ROS messages and ROS2 interfaces ##
##############################################

include(uORBToPascalCase)

if(NOT "${ROS_UORB_MSGS}" STREQUAL "")
  # Generate introspection typesupport for C and C++
  rosidl_generate_interfaces(${PROJECT_NAME} ${ROS_UORB_MSGS})
else()
  message(FATAL_ERROR "No uORB msgs found to be generated!")
endif()

ament_package()
