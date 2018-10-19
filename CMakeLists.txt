cmake_minimum_required(VERSION 3.5)

project(px4_ros_com)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

## Check if any sanitizers set
include(EnableSanitizers)

# Option to set the PX4 Firmware repository directory
option(PX4_FIRMWARE_DIR "PX4 Firmware repository directory (by default not set)" "")
if(${PX4_FIRMWARE} NOT STREQUAL "")
  set(PX4_FIRMWARE_DIR "${PX4_FIRMWARE_DIR}" CACHE INTERNAL "PX4_FIRMWARE_DIR")
endif()

# Look for the PX4 Firmware dir and get the uORB msg definitions if PX4_FIRMWARE_DIR is not set
find_package(PX4Firmware REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    geographic_msgs
    geometry_msgs
    message_generation
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

##########################
## Declare ROS messages ##
##########################

include(uORBToPascalCase)

if(NOT "${ROS_UORB_MSGS}" STREQUAL "")
    STRING(REGEX REPLACE "msg/" "" ROS_UORB_MSGS "${ROS_UORB_MSGS}")
    add_message_files(
          DIRECTORY msg
          FILES
          ${ROS_UORB_MSGS}
    )
else()
    message(FATAL_ERROR "No uORB msgs found to be generated!")
endif()

## Generate added messages
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES px4_ros_com
    CATKIN_DEPENDS geographic_msgs geometry_msgs message_generation message_runtime nav_msgs roscpp sensor_msgs std_msgs
    DEPENDS Boost
)

###########
## Build ##
###########

## Add include dirs
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Build listener
add_executable(
    sensor_combined_listener
    src/listeners/sensor_combined_listener.cpp
)
add_dependencies(sensor_combined_listener ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(sensor_combined_listener ${catkin_LIBRARIES})

## Build advertiser
# add_executable(
#     debug_vect_advertiser
#     src/advertisers/debug_vect_advertiser.cpp
# )
# add_dependencies(debug_vect_advertiser ${catkin_EXPORTED_TARGETS})
# target_link_libraries(debug_vect_advertiser ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sensor_combined_listener # debug_vect_advertiser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
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

if(CATKIN_ENABLE_TESTING)

#TODO

endif()
