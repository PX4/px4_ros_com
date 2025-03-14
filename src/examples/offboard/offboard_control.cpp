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
 * @modified_by CatScanners <https://github.com/CatScanners/find-my-kitten>
 * @date 2025-02-03
 * @details Subscribes to a trajectory publisher and sends setpoints via publish_trajectory_setpoint()
 */

 #include <px4_msgs/msg/offboard_control_mode.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vehicle_local_position.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <stdint.h>
 
 #include <std_msgs/msg/string.hpp> // to listen for incoming setpoints
 #include <chrono>
 #include <iostream>
 
 using namespace std::chrono;
 using namespace std::chrono_literals;
 using namespace px4_msgs::msg;
 
 class OffboardControl : public rclcpp::Node
 {
 public:
	 OffboardControl() : Node("offboard_control")
	 {
		 // Initial variables
		 rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		 auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
 
		 offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		 trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		 vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
 
		 custom_trajectory_subscription_ = this->create_subscription<TrajectorySetpoint>("/custom_trajectory", 10, std::bind(&OffboardControl::trajectory_setpoint_callback, this, std::placeholders::_1));
		 subscription_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,std::bind(&OffboardControl::vehicle_gps_callback, this, std::placeholders::_1));
		 current_trajectory_setpoint_.position = {0.0f, 0.0f, -3.0f};
		 offboard_setpoint_counter_ = 0;
 
		 auto timer_callback = [this]() -> void {
 
			 // Switch to offboard mode and armAFTER 10 trajectory setpoints have been sent.
			 // https://docs.px4.io/main/en/ros2/offboard_control.html#:~:text=PX4%20requires%20that%20the%20vehicle%20is%20already%20receiving%20OffboardControlMode%20messages%20before
			 if (offboard_setpoint_counter_ == 10) {
				 // Change to Offboard mode after 10 setpoints
				 this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				 // Arm the vehicle
				 // this->arm();
			 }
 
			 // offboard_control_mode needs to be paired with trajectory_setpoint
			 publish_offboard_control_mode();
			 publish_trajectory_setpoint();
 
			 // stop the counter after reaching 11, no need to count anymore.
			 if (offboard_setpoint_counter_ < 11) {
				 offboard_setpoint_counter_++;
			 }
		 };
 
		 timer_ = this->create_wall_timer(100ms, timer_callback);
	 }
 
	 void arm();
	 // void disarm();
 
 private:
 	 int print_counter_ = 0;
	 rclcpp::TimerBase::SharedPtr timer_;
 
	 rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
	 rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	 rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	 rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	 rclcpp::Subscription<TrajectorySetpoint>::SharedPtr custom_trajectory_subscription_;
 
	 std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	 uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	 TrajectorySetpoint current_trajectory_setpoint_; //!< next setpoint, where the drone should be.
	 float current_trajectory_altitude_ = 0; // current trajectory setpoint that we are sending


 
	 void publish_offboard_control_mode();
	 void publish_trajectory_setpoint();
	 void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
 
	 void vehicle_gps_callback(const VehicleLocalPosition::SharedPtr msg)
	 {
		print_counter_++;
        
        // Only print every 10th callback
        if (print_counter_ % 30 == 0)
        {
            std::cout << "z: " << msg->z
                      << " x: " << msg->x 
                      << " y: " << msg->y
                      << " yaw: " << msg->heading
					  << " dx: " << msg->vx
					  << " dy: " << msg->vy
					  << " dz: " << msg->vz
                      << std::endl;
        }

	 }
 
	 void trajectory_setpoint_callback(const TrajectorySetpoint::SharedPtr msg)
	 {
		 current_trajectory_setpoint_ = *msg;
	 }
 
 };
 
 /**
  * @brief Send a command to Arm the vehicle
  */
 void OffboardControl::arm()
 {
	 publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
 
	 RCLCPP_INFO(this->get_logger(), "Arm command send");
 }
 
 /**
  * @brief Send a command to Disarm the vehicle
  */
 /*void OffboardControl::disarm()
 {
	 publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
 
	 RCLCPP_INFO(this->get_logger(), "Disarm command send");
 }*/
 
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
  * @brief Publish a trajectory setpoint
  */
 void OffboardControl::publish_trajectory_setpoint()
 {	
	 // LOOK AT WAYPOINTS: https://github.com/PX4/px4_msgs/blob/main/msg/TrajectoryWaypoint.msg
	 if (current_trajectory_setpoint_.position.size() == 3) { // x, y and z coordinate received.
		 /* RCLCPP_INFO(this->get_logger(), "Published trajectory: [%.2f, %.2f, %.2f]",
					 current_trajectory_setpoint_.position[0], current_trajectory_setpoint_.position[1], current_trajectory_setpoint_.position[2]); */
		 current_trajectory_setpoint_.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		 trajectory_setpoint_publisher_->publish(current_trajectory_setpoint_);
	 } else {
		 RCLCPP_WARN(this->get_logger(), "No valid trajectory setpoint received!");
	 }
 }
 
 /**
  * @brief Publish vehicle commands
  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
  * @param param1    Command parameter 1
  * @param param2    Command parameter 2
  */
 void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
 {
	 VehicleCommand msg{};
	 msg.param1 = param1;
	 msg.param2 = param2;
	 msg.command = command;
	 msg.target_system = 1;
	 msg.target_component = 1;
	 msg.source_system = 1;
	 msg.source_component = 1;
	 msg.from_external = true;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 vehicle_command_publisher_->publish(msg);
	 // Check if successfull? https://docs.px4.io/main/en/msg_docs/VehicleCommandAck.html
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
 