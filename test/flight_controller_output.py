#!/usr/bin/env python

################################################################################
#
#   Copyright 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

# This script launches the microRTPS bridge pipeline to verify if there is
# data being exchanged between the flight controller (in SITL) and companion
# computer side, with its DDS participants

import argparse
import os
import time
from sys import exit
from subprocess import call, CalledProcessError, check_output, DEVNULL, Popen, STDOUT

if __name__ == "__main__":

    default_px4_ros_com_install_dir = "$HOME/PX4/px4_ros2_ws/install"
    default_px4_dir = "$HOME/PX4/Firmware"
    default_px4_target = "iris_rtps"
    default_listener_topic = "sensor_combined"

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--px4-firmware-dir", dest='px4_dir', type=str,
                        help="PX4 Firmware dir, defaults to $HOME/PX4/Firmware/", default=default_px4_dir)
    parser.add_argument("-t", "--px4-build-target", dest='px4_target', type=str,
                        help="PX4 SITL target, defaults to iris_rtps", default=default_px4_target)
    parser.add_argument("-l", "--listen", dest='listener_topic', type=str,
                        help="ROS2 topic to test, defaults to 'sensor_combined'", default=default_listener_topic)

    # Parse arguments
    args = parser.parse_args()
    px4_dir = args.px4_dir
    px4_target = args.px4_target
    listener = args.listener_topic

    # get ROS distro
    ros_distro = check_output("rosversion -d", shell=True)
    print("\033[34m" + "\n-------------- PX4 MICRORTPS COMMUNICATION TESTS --------------" + "\033[0m" + "\033[5m" + "\n\n-- Running test over" + "\033[93m" + " ROS2 " +
          str(ros_distro.strip().decode("utf-8")).capitalize() + "\033[0m")

    # launch the microRTPS agent
    print("\n-- Starting microRTPS bridge agent...\n")
    call("micrortps_agent -t UDP &", shell=True, stderr=STDOUT)

    # waits for the agent to load
    time.sleep(3)

    # launch PX4 SITL daemon, in headless mode
    print("\n-- Starting the PX4 SITL daemon and Gazebo (without GUI)...\n")
    call("cd " + px4_dir + " && (NO_PXH=1 HEADLESS=1 make px4_sitl_rtps gazebo_" +
         px4_target + " &) && cd " + os.getcwd(), shell=True, stderr=DEVNULL)

    # waits for PX4 daemon and Gazebo to load
    time.sleep(10)

    # launch the specified test
    if (listener == "sensor_combined"):
        test_result = call(
            "python3 test_sensor_combined_topic_out.py", stderr=STDOUT, shell=True,
            universal_newlines=True)

    call("killall gzserver micrortps_agent px4 ros2",
         shell=True, stdout=DEVNULL, stderr=DEVNULL)

    print("\033[34m" + "-------------- TEST RESULTS --------------" + "\033[0m")
    if (test_result):
        print("\033[91m" + "[FAILED]\tFlight controller output test failed! Failed to get data from the '" +
              listener + "' uORB topic" + "\033[0m" + "\033[34m" + "\n------------------------------------------\n" + "\033[0m")
        exit(1)
    else:
        print("\033[92m" + "[SUCCESS]\tFlight controller output test successfull! Successfully retrieved data from the '" +
              listener + "' uORB topic" + "\033[0m" + "\033[34m" + "\n------------------------------------------\n" + "\033[0m")
        exit(0)
