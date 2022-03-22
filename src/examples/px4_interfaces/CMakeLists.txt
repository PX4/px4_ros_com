cmake_minimum_required(VERSION 3.5)

project(px4_interfaces)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB px4_interfaces RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

file(GLOB ROS_MSG_DIR_LIST "${MSGS_DIR}/*.msg")

set(ROS_MSG_DIR_LIST "${ROS_MSG_DIR_LIST}" CACHE INTERNAL "ROS_MSG_DIR_LIST")

if(NOT "${px4_interfaces}" STREQUAL "")
  # Generate introspection typesupport for C and C++ and IDL files
  rosidl_generate_interfaces(${PROJECT_NAME}
                             ${px4_interfaces}
                             DEPENDENCIES
                             builtin_interfaces
                             ADD_LINTER_TESTS)
else()
  message(FATAL_ERROR "No PX4 msgs found to generate interfaces to!")
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
