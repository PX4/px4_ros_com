cmake_minimum_required(VERSION 3.5)

project(px4_ros_com)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Look for the PX4 Firmware dir and get the uORB msg definitions
find_package(PX4Firmware REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

find_package(PythonInterp REQUIRED)

##########################
## Declare ROS messages ##
##########################

include(uORBToPascalCase)

if(NOT "${ROS_UORB_MSGS}" STREQUAL "")
    rosidl_generate_interfaces(${PROJECT_NAME}
      ${ROS_UORB_MSGS}
      DEPENDENCIES
      builtin_interfaces
    )
else()
    message(FATAL_ERROR "No uORB msgs found to be generated!")
endif()

####################################
## Generate micro-RTPS agent code ##
####################################

include(GenerateMicroRTPSAgent)

###################################
## ament specific configuration ##
###################################

###########
## Build ##
###########

ament_export_dependencies(rosidl_default_runtime)

#############
## Install ##
#############

#TODO

#############
## Testing ##
#############

#TODO

ament_package()
