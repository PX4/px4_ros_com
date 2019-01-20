#################################################################################
#
# Copyright (c) 2018-2019 PX4 Pro Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name PX4 nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##################################################################################

##################################################################################
# Try to find the PX4 Firmware and the uORB msg descriptions dirs
#
# The following are set after configuration is done:
#       - PX4_FIRMWARE_FOUND
#       - PX4_FIRMWARE_REPO_DIR
#       - PX4_FIRMWARE_SRC_DIR
#       - PX4_FIRMWARE_INCLUDE_DIR
#       - PX4_FIRMWARE_MSG_DIR
#       - PX4_FIRMWARE_GIT_ANNOTATED_TAG
#       - PX4_FIRMWARE_GIT_COMMIT_HASH
##################################################################################

# search hints
set(PX4_FIRMWARE_EXTRA_SEARCH_HINTS
    ${PX4_FIRMWARE_DIR}/src
    ${CMAKE_SOURCE_DIR}/Firmware/src
    ${CMAKE_SOURCE_DIR}/PX4/Firmware/src
    ${CATKIN_DEVEL_PREFIX}/Firmware/src
    ${CATKIN_DEVEL_PREFIX}/PX4/Firmware/src
    ../Firmware/src)

# search paths
set(PX4_FIRMWARE_EXTRA_SEARCH_PATHS $ENV{HOME}/Firmware/src
    $ENV{HOME}/PX4/Firmware/src)

# look for in the hints first
find_path(PX4_FIRMWARE_INCLUDE_DIRS
          NAMES px4.h
          PATH_SUFFIXES include
          HINTS ${PX4_FIRMWARE_EXTRA_SEARCH_HINTS} ENV HOME
          NO_DEFAULT_PATH)

# look for in the hard-coded paths
find_path(PX4_FIRMWARE_INCLUDE_DIRS
          NAMES px4.h
          PATH_SUFFIXES include
          PATHS ${PX4_FIRMWARE_EXTRA_SEARCH_PATHS} ENV HOME
          NO_CMAKE_PATH
          NO_CMAKE_ENVIRONMENT_PATH
          NO_SYSTEM_ENVIRONMENT_PATH
          NO_CMAKE_SYSTEM_PATH)

get_filename_component(PX4_FIRMWARE_SRC_DIR "${PX4_FIRMWARE_INCLUDE_DIRS}"
                       DIRECTORY)
get_filename_component(PX4_FIRMWARE_REPO_DIR "${PX4_FIRMWARE_SRC_DIR}"
                       DIRECTORY)

set(PX4_FIRMWARE_INCLUDE_DIR ${PX4_FIRMWARE_INCLUDE_DIRS})
set(PX4_FIRMWARE_MSG_DIR "${PX4_FIRMWARE_REPO_DIR}/msg")

message(STATUS "PX4 Firmware repo dir: ${PX4_FIRMWARE_REPO_DIR}")
message(STATUS "PX4 Firmware src dir: ${PX4_FIRMWARE_SRC_DIR}")
message(STATUS "PX4 Firmware include dir: ${PX4_FIRMWARE_INCLUDE_DIR}")
message(STATUS "PX4 Firmware msg dir: ${PX4_FIRMWARE_MSG_DIR}")

# Get the latest annotated tag of the working branch
execute_process(COMMAND git describe --abbrev=0
                WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
                OUTPUT_VARIABLE PX4_FIRMWARE_GIT_ANNOTATED_TAG
                OUTPUT_STRIP_TRAILING_WHITESPACE)
# Get the latest abbreviated commit tag of the working branch
execute_process(COMMAND git log -1 --format=%h
                WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
                OUTPUT_VARIABLE PX4_FIRMWARE_GIT_COMMIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE)

# Set the Firmware version (release | commit)
set(PX4_FIRMWARE_VERSION
    "${PX4_FIRMWARE_GIT_ANNOTATED_TAG} | ${PX4_FIRMWARE_GIT_COMMIT_HASH}")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(PX4Firmware
                                  FOUND_VAR
                                  PX4Firmware_FOUND
                                  REQUIRED_VARS
                                  PX4_FIRMWARE_REPO_DIR
                                  PX4_FIRMWARE_INCLUDE_DIR
                                  PX4_FIRMWARE_MSG_DIR
                                  VERSION_VAR
                                  PX4_FIRMWARE_VERSION)

mark_as_advanced(PX4_FIRMWARE_REPO_DIR
                 PX4_FIRMWARE_SRC_DIR
                 PX4_FIRMWARE_INCLUDE_DIR
                 PX4_FIRMWARE_MSG_DIR
                 PX4_FIRMWARE_GIT_ANNOTATED_TAG
                 PX4_FIRMWARE_GIT_COMMIT_HASH)
