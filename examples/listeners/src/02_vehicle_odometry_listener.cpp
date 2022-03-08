/****************************************************************************
 *
 * Copyright 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Vehicle GPS position uORB topic listener example
 * @file 02_vehicle_global_position_listener.cpp
 * @addtogroup examples
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 * @author Bastian Jäger <bastian@auterion.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

/**
 * @brief Vehicle Odometry uORB topic data callback
 */
class VehicleOdometryListener : public rclcpp::Node
{
public:
    explicit VehicleOdometryListener() : Node("vehicle_global_position_listener") {
        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "fmu/vehicle_odometry/out", 10,
            [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
            std::cout << "\n\n\n\n";
            std::cout << "RECEIVED VEHICLE ODOMETRY DATA"   << std::endl;
            std::cout << "=================================="   << std::endl;
            std::cout << "timestamp: " << msg->timestamp    << std::endl;
            std::cout << "position: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
            std::cout << "velocity: " << msg->vx << ", " << msg->vy << ", " << msg->vz << std::endl;
            std::cout << "orientation: " << msg->q[0] << ", " << msg->q[1] << ", " << msg->q[2] << ", " << msg->q[3] << std::endl;
            std::cout << "angular vel: " << msg->rollspeed << ", " << msg->pitchspeed << ", " << msg->yawspeed << std::endl;
        });
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting vehicle_odometry listener node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleOdometryListener>());

    rclcpp::shutdown();
    return 0;
}
