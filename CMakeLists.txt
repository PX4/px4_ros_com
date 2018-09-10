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

# Add the python scripts dir from PX4 msg/tools to PYTHONPATH
set(ENV{PYTHONPATH} "$ENV{PYTHONPATH}:${PX4_FIRMWARE_MSG_DIR}/tools/")

# Check if the RTPS ID's mapper yaml file exists and if yes, set it as a environment variables
if(EXISTS ${PX4_FIRMWARE_MSG_DIR}/tools/uorb_rtps_message_ids.yaml)
    set(ENV{RTPS_ID_YAML_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/msg/templates/uorb_rtps_message_ids.yaml)
    message(STATUS "RTPS msg ID yaml file: $ENV{RTPS_ID_YAML_FILE}")

    # Parse the RTPS IDs for each uORB msg into the counterparts for ROS
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/PX42ROSRTPSIds.py
            --input-file ${PX4_FIRMWARE_MSG_DIR}/tools/uorb_rtps_message_ids.yaml
            --output-file $ENV{RTPS_ID_YAML_FILE}
        )
else()
    message(FATAL_ERROR "RTPS msg ID yaml file ${RTPS_ID_YAML_FILE} not found!")
endif()

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} ${PX4_FIRMWARE_MSG_DIR}/tools/generate_microRTPS_bridge.py
        --fastrtpsgen-dir $ENV{FASTRTPSGEN_DIR}
        --send ${ROS_UORB_MSGS_DIR}
        --receive ${ROS_UORB_MSGS_DIR}
        --topic-msg-dir ${CMAKE_CURRENT_SOURCE_DIR}/msg
        --urtps-templates-dir "/templates"
        --agent
        --agent-outdir ${CMAKE_CURRENT_SOURCE_DIR}/src/micrortps_agent
        --package ${PROJECT_NAME}
    )

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
