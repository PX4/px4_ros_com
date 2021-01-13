/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
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
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
		    this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		position_setpoint_triplet_publisher_ =
		    this->create_publisher<PositionSetpointTriplet>("PositionSetpointTriplet_PubSubTopic", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
#else
		offboard_control_mode_publisher_ =
		    this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		position_setpoint_triplet_publisher_ =
		    this->create_publisher<PositionSetpointTriplet>("PositionSetpointTriplet_PubSubTopic");
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			// define common timestamp
			auto timestamp = time_point_cast<microseconds>(steady_clock::now()).time_since_epoch().count();

			if (offboard_setpoint_counter_ == 100) {
				// Change to Offboard mode after 100 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, timestamp, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            // offboard_control_mode needs to be paired with position_setpoint_triplet
			publish_offboard_control_mode(timestamp);
			publish_position_setpoint_triplet(timestamp);

            // stop the counter after reaching 100
			if (offboard_setpoint_counter_ < 101) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(10ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<PositionSetpointTriplet>::SharedPtr position_setpoint_triplet_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode(const long int& timestamp) const;
	void publish_position_setpoint_triplet(const long int& timestamp) const;
	void publish_vehicle_command(uint16_t command, const long int& timestamp, float param1 = 0.0,
				     float param2 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	auto timestamp = time_point_cast<microseconds>(steady_clock::now()).time_since_epoch().count();

	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	auto timestamp = time_point_cast<microseconds>(steady_clock::now()).time_since_epoch().count();

	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 * @param timestamp Timestamp of the setpoint triplet
 */
void OffboardControl::publish_offboard_control_mode(const long int& timestamp) const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp;
	msg.ignore_thrust = true;
	msg.ignore_attitude = true;
	msg.ignore_bodyrate_x = true;
	msg.ignore_bodyrate_y = true;
	msg.ignore_bodyrate_z = true;
	msg.ignore_position = false;
	msg.ignore_velocity = true;
	msg.ignore_acceleration_force = true;
	msg.ignore_alt_hold = true;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish position setpoint triplets.
 *        For this example, it sends position setpoint triplets to make the
 *        vehicle hover at 5 meters.
 * @param timestamp Timestamp of the setpoint triplet
 */
void OffboardControl::publish_position_setpoint_triplet(const long int& timestamp) const {
	PositionSetpointTriplet msg{};
	msg.timestamp = timestamp;
	msg.current.timestamp = timestamp;
	msg.current.type = PositionSetpoint::SETPOINT_TYPE_POSITION;
	msg.current.x = 0.0;
	msg.current.y = 0.0;
	msg.current.z = -5.0;
	msg.current.yaw = 1.5707963268;
	msg.current.cruising_speed = -1.0;
	msg.current.position_valid = true;
	msg.current.yaw_valid = true;
	msg.current.alt_valid = true;
	msg.current.valid = true;

	position_setpoint_triplet_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param timestamp Timestamp of the command
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, const long int& timestamp, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
