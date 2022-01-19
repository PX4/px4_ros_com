# /****************************************************************************
# *
# * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL(eProsima).
# *           2018 PX4 Pro Development Team. All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
#     *
#     * 1. Redistributions of source code must retain the above copyright notice, this
#     * list of conditions and the following disclaimer.
#     *
#     * 2. Redistributions in binary form must reproduce the above copyright notice,
#     * this list of conditions and the following disclaimer in the documentation
#     * and / or other materials provided with the distribution.
#     *
#     * 3. Neither the name of the copyright holder nor the names of its contributors
#     * may be used to endorse or promote products derived from this software without
#     * specific prior written permission.
#     *
#     * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#     * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#     * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#     * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#     * CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#                             * SUBSTITUTE GOODS OR SERVICES
#                             LOSS OF USE, DATA, OR PROFITS
#                             OR BUSINESS
#                             * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#     * CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE)
#     * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     * POSSIBILITY OF SUCH DAMAGE.
#     *
#     ****************************************************************************/

# /**
# * @ brief Sensor Combined uORB topic listener example
# * @ file sensor_combined_listener_py.py
# * @ addtogroup examples
# * @ author < hanghaotian@gmail.com >
# * @ author Haotian Hang
# */


# /**
# * @ brief Sensor Combined uORB topic data callback
# */

import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined


class SensorCombinedListener(Node):
    def __init__(self):
        super().__init__('sensor_combined_listener')
        self.subscription = self.create_subscription(
            SensorCombined,
            "fmu/sensor_combined/out",
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("RECEIVED SENSOR COMBINED DATA")
        print("=============================")
        print("ts: ", msg.timestamp)
        print("gyro_rad[0]: ", msg.gyro_rad[0])
        print("gyro_rad[1]: ", msg.gyro_rad[1])
        print("gyro_rad[2]: ", msg.gyro_rad[2])
        print("accelerometer_timestamp_relative: ",
              msg.accelerometer_timestamp_relative)
        print("accelerometer_m_s2[0]: ", msg.accelerometer_m_s2[0])
        print("accelerometer_m_s2[1]: ", msg.accelerometer_m_s2[1])
        print("accelerometer_m_s2[2]: ", msg.accelerometer_m_s2[2])
        print("accelerometer_integral_dt: ", msg.accelerometer_integral_dt)


def main(args=None):
    rclpy.init(args=args)
    print("Starting sensor_combined listener node...")
    sensor_combined_listener = SensorCombinedListener()
    rclpy.spin(sensor_combined_listener)
    sensor_combined_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
