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

# This script tests the output of 'ros2 topic echo /SensorCombined_PubSubTopic'
# to evaluate if there's data output coming out of it. If the log output file
# is empty, means that there's no output on the topic, so the test fails

import os
from sys import exit
from subprocess import call, check_output, DEVNULL
from time import sleep

if __name__ == "__main__":
    px4_bin_dir = os.environ["PX4_BIN_DIR"] if "PX4_BIN_DIR" in os.environ else "$HOME/PX4/Firmware/build/px4_sitl_rtps/bin"
    test_cmd = "/bin/bash -c '" + \
        os.path.join(os.environ["PX4_BIN_DIR"],
                     "px4-listener") + " debug_vect'"

    print("\n\033[93m-- 'debug_vect' uORB data test launched:\033[0m")
    # start the debug_vect ROS 2 avertiser
    call("/bin/bash -c 'debug_vect_advertiser &'", shell=True)

    sleep(3)

    # call the 'listener' command for the 'debug_vect' uORB topic
    output = check_output(test_cmd, shell=True, universal_newlines=True)

    call("killall debug_vect_advertiser",
         shell=True, stdout=DEVNULL, stderr=DEVNULL)

    if output and "never published" not in output:
        print(
            "\n\033[42m-- Successfully obtained data on 'debug_vect' uORB topic. microRTPS bridge is up! Output:\033[0m")
        print("\033[97m" + output + "\033[0m")
        exit(0)
    else:
        print(
            "\n\033[41m-- Something went wrong. Please check if the microRTPS bridge is functioning correctly.\033[0m\n")
        exit(1)
