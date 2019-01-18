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
# Run a script to convert the uORB msg definitions files to PascalCase so to be
# processed by generate_messages() on catkin
##################################################################################

set(ROS_UORB_MSGS_LIST "")

function(glob_generate target file_glob)
  file(GLOB uorb_glob_files ${file_glob})
  set(UORB_MSGS_LIST)
  set(PX4_ROS_COM_MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg/")
  file(MAKE_DIRECTORY ${PX4_ROS_COM_MSG_DIR})
  execute_process(COMMAND ${PYTHON_EXECUTABLE}
                          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/uORB2ROSMsgs.py
                          ${PROJECT_NAME} "${PX4_FIRMWARE_MSG_DIR}/"
                          ${PX4_ROS_COM_MSG_DIR}
                  WORKING_DIRECTORY ${PX4_FIRMWARE_REPO_DIR}
                  OUTPUT_QUIET)
  file(GLOB ros_glob_files ${PX4_ROS_COM_MSG_DIR}*.msg)
  message(STATUS "Added the following msgs to ROS2 msg generation")
  foreach(ros_glob_file ${ros_glob_files})
    get_filename_component(filename_we ${ros_glob_file} NAME_WE)
    list(APPEND ROS_UORB_MSGS_NAMES "${filename_we}")
    get_filename_component(filename ${ros_glob_file} NAME)
    list(APPEND ROS_UORB_MSGS_LIST "msg/${filename}")
    list(APPEND ROS_UORB_MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg/${filename}")
    message(STATUS "\t${filename}")
  endforeach()
  set(ROS_UORB_NAMES "${ROS_UORB_MSGS_NAMES}" CACHE INTERNAL "ROS_UORB_NAMES")
  set(ROS_UORB_MSGS "${ROS_UORB_MSGS_LIST}" CACHE INTERNAL "ROS_UORB_MSGS")
  set(ROS_UORB_MSGS_DIR "${ROS_UORB_MSGS_DIR}"
      CACHE INTERNAL "ROS_UORB_MSGS_DIR")
endfunction()

glob_generate(models_gen ${PX4_FIRMWARE_MSG_DIR}*.msg)
