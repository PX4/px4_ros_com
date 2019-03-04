cmake_minimum_required(VERSION 3.5)

project(px4_ros_com)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Check if any sanitizers set
include(EnableSanitizers)

# Option to set the PX4 Firmware repository directory
option(PX4_FIRMWARE_DIR "PX4 Firmware repository directory (by default not set)"
       "")
if(${PX4_FIRMWARE} NOT STREQUAL "")
  set(PX4_FIRMWARE_DIR "${PX4_FIRMWARE_DIR}" CACHE INTERNAL "PX4_FIRMWARE_DIR")
endif()

# Look for the PX4 Firmware dir if PX4_FIRMWARE_DIR is not set
# find_package(PX4Firmware)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(px4_msgs REQUIRED)

find_package(PythonInterp REQUIRED)

################################
# Generate Fast RTPS IDL files #
################################

# Builds lists of ROS messages and IDL files
get_filename_component(MSGS_DIR "../" ABSOLUTE BASE_DIR "${px4_msgs_DIR}")

set(MSGS_DIR "${MSGS_DIR}/msg")
file(GLOB DDS_IDL_DIR_LIST ${MSGS_DIR}/*.idl)
file(GLOB ROS_MSG_DIR_LIST ${MSGS_DIR}/*.msg)

set(DDS_IDL_DIR_LIST "${DDS_IDL_DIR_LIST}" CACHE INTERNAL "DDS_IDL_DIR_LIST")
set(ROS_MSG_DIR_LIST "${ROS_MSG_DIR_LIST}" CACHE INTERNAL "ROS_MSG_DIR_LIST")

rosidl_generate_dds_interfaces(${PROJECT_NAME}__fastrtps_idl
                                IDL_FILES
                                ${ROS_MSG_DIR_LIST}
                                OUTPUT_SUBFOLDERS
                                "dds_fastrtps")
set(DDS_IDL_FILES "" CACHE INTERNAL "DDS_IDL_FILES")
set(DDS_IDL_BASE_PATH "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl")
foreach(_idl_file ${ROS_MSG_DIR_LIST})
  get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_extension "${_idl_file}" EXT)
  get_filename_component(_name "${_idl_file}" NAME_WE)
  if(_extension STREQUAL ".msg")
    list(
        APPEND
        DDS_IDL_FILES
        "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/${_name}_.idl"
        )
  elseif(_extension STREQUAL ".srv")
    list(
        APPEND
        DDS_IDL_FILES
        "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/Sample_${_name}_Request_.idl"
        )
    list(
        APPEND
        DDS_IDL_FILES
        "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/Sample_${_name}_Response_.idl"
        )
  endif()
endforeach()

###################################
# Generate micro-RTPS agent code ##
###################################

include(GenerateMicroRTPSAgent)
add_dependencies(${PROJECT_NAME}__fastrtps_idl micrortps_agent)

#################
# Setup targets #
#################

include_directories(include ${rclcpp_INCLUDE_DIRS})

# Build micro-RTPS agent code
add_executable(micrortps_agent ${MICRORTPS_AGENT_FILES})
target_link_libraries(micrortps_agent fastrtps fastcdr)

# Build listener
add_executable(sensor_combined_listener
               src/listeners/sensor_combined_listener.cpp)
add_dependencies(sensor_combined_listener micrortps_agent)

# Build advertiser
add_executable(debug_vect_advertiser src/advertisers/debug_vect_advertiser.cpp)
add_dependencies(debug_vect_advertiser micrortps_agent)

#################################
# ament specific configuration ##
#################################

# Add listener dependencies
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)

# Add advertiser dependencies
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)

# Export information to downstream packages
ament_export_dependencies(rclcpp rosidl_default_runtime)
ament_export_include_directories(include ${px4_msgs_INCLUDE_DIRS})
ament_export_libraries()

############
# Install ##
############

# Install artifacts
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
install(TARGETS micrortps_agent sensor_combined_listener debug_vect_advertiser
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# install ROS2<->ROS1 mapping rules
install(FILES px4_ros_com_mapping_rules.yaml DESTINATION share/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

############
# Testing ##
############

# TODO

ament_package()
