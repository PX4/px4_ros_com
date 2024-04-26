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
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

const rmw_qos_profile_t rmw_qos_profile_offboard_pub =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  5,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_offboard_pub;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", qos);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 50) {
				// Change to Offboard mode after 50 setpoints (1s)
				this->engage_offBoard_mode();
				
				// Arm the vehicle
				this->arm();
			}
			if (offboard_setpoint_counter_ == 550){
				// Land and cancel timer after (11s)
				this->land();

				this->timer_->cancel();
			}
			if (offboard_setpoint_counter_ < 550) {

				// offboard_control_mode needs to be paired with trajectory_setpoint
				publish_offboard_control_mode();
				publish_trajectory_setpoint();

				offboard_setpoint_counter_++;
			}
		};
		// position controller runs at 50Hz
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

	void arm();
	void disarm();
	void land();
	void engage_offBoard_mode();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(VehicleCommand msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	VehicleCommand msg{};

	msg.param1 = 1;
	msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	VehicleCommand msg{};

	msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to Land the vehicle
 */
void OffboardControl::land()
{
	VehicleCommand msg{};

	msg.command = VehicleCommand::VEHICLE_CMD_NAV_LAND;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Land command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};

	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Engage the offboard control mode.
 */
void OffboardControl::engage_offBoard_mode()
{
	VehicleCommand msg{};

	msg.param1 = 1;
	msg.param2 = 6;
	msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
	msg.target_system = 1;
    	msg.target_component = 1;
    	msg.source_system = 1;
    	msg.source_component = 1;
    	msg.from_external = true;

	publish_vehicle_command(msg);

	RCLCPP_INFO(this->get_logger(), "Offboard mode command sent");
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = -3.14; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command message
 */
void OffboardControl::publish_vehicle_command(VehicleCommand msg)
{
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
