# Try to find the PX4 Firmware and the uORB msg descriptions dirs
#
# The following are set after configuration is done:
#  - PX4_FIRMWARE_FOUND
#  - PX4_FIRMWARE_REPO_DIR
#  - PX4_FIRMWARE_SRC_DIR
#  - PX4_FIRMWARE_INCLUDE_DIR
#  - PX4_FIRMWARE_MSG_DIR
#  - PX4_FIRMWARE_GIT_ANNOTATED_TAG
#  - PX4_FIRMWARE_GIT_COMMIT_HASH

# search hints
set(PX4_FIRMWARE_EXTRA_SEARCH_HINTS
    ${CMAKE_SOURCE_DIR}/Firmware/src
    ${CMAKE_SOURCE_DIR}/PX4/Firmware/src
    ${CATKIN_DEVEL_PREFIX}/Firmware/src
    ${CATKIN_DEVEL_PREFIX}/PX4/Firmware/src
    ../Firmware/src
    )

# search paths
set(PX4_FIRMWARE_EXTRA_SEARCH_PATHS
    $ENV{HOME}/Firmware/src
    $ENV{HOME}/PX4/Firmware/src
    )


# look for in the hints first
find_path(PX4_FIRMWARE_INCLUDE_DIRS
    NAMES px4.h
    PATH_SUFFIXES include
    HINTS ${PX4_FIRMWARE_EXTRA_SEARCH_HINTS} ENV HOME
    NO_DEFAULT_PATH
    )

# look for in the hard-coded paths
find_path(PX4_FIRMWARE_INCLUDE_DIRS
    NAMES px4.h
    PATH_SUFFIXES include
    PATHS ${PX4_FIRMWARE_EXTRA_SEARCH_PATHS} ENV HOME
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
    )

get_filename_component(PX4_FIRMWARE_SRC_DIR "${PX4_FIRMWARE_INCLUDE_DIRS}" DIRECTORY)
get_filename_component(PX4_FIRMWARE_REPO_DIR "${PX4_FIRMWARE_SRC_DIR}" DIRECTORY)

set(PX4_FIRMWARE_INCLUDE_DIR ${PX4_FIRMWARE_INCLUDE_DIRS})
set(PX4_FIRMWARE_MSG_DIR "${PX4_FIRMWARE_REPO_DIR}/msg")

message(STATUS "PX4 Firmware repo dir: ${PX4_FIRMWARE_REPO_DIR}")
message(STATUS "PX4 Firmware src dir: ${PX4_FIRMWARE_SRC_DIR}")
message(STATUS "PX4 Firmware include dir: ${PX4_FIRMWARE_INCLUDE_DIR}")
message(STATUS "PX4 Firmware msg dir: ${PX4_FIRMWARE_MSG_DIR}")

# Get the latest annotated tag of the working branch
execute_process(
  COMMAND git describe --abbrev=0
  WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
  OUTPUT_VARIABLE PX4_FIRMWARE_GIT_ANNOTATED_TAG
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
# Get the latest abbreviated commit tag of the working branch
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
  OUTPUT_VARIABLE PX4_FIRMWARE_GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Set the Firmware version (release | commit)
set(PX4_FIRMWARE_VERSION "${PX4_FIRMWARE_GIT_ANNOTATED_TAG} | ${PX4_FIRMWARE_GIT_COMMIT_HASH}")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(PX4Firmware FOUND_VAR PX4Firmware_FOUND
                                  REQUIRED_VARS PX4_FIRMWARE_REPO_DIR PX4_FIRMWARE_INCLUDE_DIR
				  PX4_FIRMWARE_MSG_DIR VERSION_VAR PX4_FIRMWARE_VERSION
				  )

mark_as_advanced(PX4_FIRMWARE_REPO_DIR PX4_FIRMWARE_SRC_DIR
                 PX4_FIRMWARE_INCLUDE_DIR PX4_FIRMWARE_MSG_DIR
		 PX4_FIRMWARE_GIT_ANNOTATED_TAG
		 PX4_FIRMWARE_GIT_COMMIT_HASH
		 )
