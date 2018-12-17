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
option(PX4_FIRMWARE_DIR "PX4 Firmware repository directory (by default not set)" "")
if(${PX4_FIRMWARE} NOT STREQUAL "")
  set(PX4_FIRMWARE_DIR "${PX4_FIRMWARE_DIR}" CACHE INTERNAL "PX4_FIRMWARE_DIR")
endif()

# Look for the PX4 Firmware dir and get the uORB msg definitions if PX4_FIRMWARE_DIR is not set
find_package(PX4Firmware REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fastrtps REQUIRED)
find_package(fastcdr REQUIRED)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

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

#################
# Setup targets #
#################

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Build micro-RTPS agent code
add_executable(micrortps_agent ${MICRORTPS_AGENT_FILES})
target_link_libraries(micrortps_agent fastrtps fastcdr)

# Build listener
add_executable(
    sensor_combined_listener
    src/listeners/sensor_combined_listener.cpp
)
add_dependencies(sensor_combined_listener micrortps_agent)

# Build advertiser
add_executable(
    debug_vect_advertiser
    src/advertisers/debug_vect_advertiser.cpp
)
add_dependencies(debug_vect_advertiser micrortps_agent)

###################################
## ament specific configuration ##
###################################

# Finds the relevant generated C++ code for the msgs generated from rosidl_generator_dds_idl
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(
        sensor_combined_listener
        ${PROJECT_NAME} ${typesupport_impl}
    )
    rosidl_target_interfaces(
        debug_vect_advertiser
        ${PROJECT_NAME} ${typesupport_impl}
    )
endforeach()

# Add listener dependencies
ament_target_dependencies(
    sensor_combined_listener
    rclcpp
)

# Add advertiser dependencies
ament_target_dependencies(
    debug_vect_advertiser
    rclcpp
)

# Export information to downstream packages
ament_export_dependencies(
  rclcpp
  rosidl_default_runtime
)
ament_export_include_directories(include)
ament_export_libraries()

#############
## Install ##
#############

# Install artifacts
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include
  DESTINATION include
)
install(
  TARGETS micrortps_agent sensor_combined_listener debug_vect_advertiser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install ROS2<->ROS1 mapping rules
install(
  FILES px4_ros_com_mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

#TODO

ament_package()
