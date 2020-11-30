#include <offboard_example.h>
#include <chrono>
#include <iostream>

OffboardExample::OffboardExample(rclcpp::Node& node)
{
  this->offboard_control_mode_publisher = node.create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
  this->position_setpoint_triplet_publisher = node.create_publisher<px4_msgs::msg::PositionSetpointTriplet>("PositionSetpointTriplet_PubSubTopic", 10);
  this->vehicle_command_publisher = node.create_publisher<px4_msgs::msg::VehicleCommand>("VehicleCommand_PubSubTopic", 10);
  this->timesync_subscriber = node.create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10, std::bind(&OffboardExample::timesync_callback, this, std::placeholders::_1));
  this->vehicle_control_mode_subscriber = node.create_subscription<px4_msgs::msg::VehicleControlMode>("VehicleControlMode_PubSubTopic", 10, std::bind(&OffboardExample::vehicle_control_mode_callback, this, std::placeholders::_1));
  this->timestamp = 0;
  this->px4_stream_count = 0;
  this->twist = control::msg::Twist{};
  this->vehicle_control_mode = px4_msgs::msg::VehicleControlMode{};
  this->stream_timer = node.create_wall_timer(std::chrono::milliseconds(100), std::bind(&OffboardExample::px4_offboard_stream, this));
}

void OffboardExample::arm() const
{
  std::cout << "OffboardExample::arm()" << std::endl;
  this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void OffboardExample::disarm() const
{
  std::cout << "OffboardExample::disarm()" << std::endl;
  this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void OffboardExample::set_velocity(const control::msg::Twist& twist)
{
  this->twist = twist;
}

void OffboardExample::timesync_callback(const px4_msgs::msg::Timesync::UniquePtr msg)
{
  if (msg->sys_id == 1)
  {
    this->timestamp = msg->timestamp;
  }
}

void OffboardExample::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  this->vehicle_control_mode = *msg;
}

void OffboardExample::publish_offboard_control_mode() const
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = this->timestamp;
  msg.ignore_thrust = true;
  msg.ignore_attitude = true;
  msg.ignore_bodyrate_x = true;
  msg.ignore_bodyrate_y = true;
  msg.ignore_bodyrate_z = false;
  msg.ignore_position = true;
  msg.ignore_velocity = false;
  msg.ignore_acceleration_force = true;
  msg.ignore_alt_hold = true;
  this->offboard_control_mode_publisher->publish(msg);
}

void OffboardExample::publish_position_setpoint_triplet() const
{
  px4_msgs::msg::PositionSetpointTriplet msg{};
  msg.timestamp = this->timestamp;
  msg.current.timestamp = this->timestamp;
  msg.current.valid = true;
  msg.current.type = px4_msgs::msg::PositionSetpoint::SETPOINT_TYPE_VELOCITY;
  msg.current.vx = this->twist.linear.x;
  msg.current.vy = this->twist.linear.y;
  msg.current.vz = this->twist.linear.z;
  msg.current.velocity_valid = true;
  msg.current.velocity_frame = px4_msgs::msg::PositionSetpoint::VELOCITY_FRAME_LOCAL_NED;
  msg.current.yawspeed = this->twist.yaw;
  msg.current.yawspeed_valid = true;
  this->position_setpoint_triplet_publisher->publish(msg);
}

void OffboardExample::publish_vehicle_command(uint16_t command, float param1, float param2) const
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = this->timestamp;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  this->vehicle_command_publisher->publish(msg);
}

void OffboardExample::px4_offboard_stream()
{
  if (this->px4_stream_count == 100)
  {
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  }
  this->publish_offboard_control_mode();
  this->publish_position_setpoint_triplet();
  this->px4_stream_count++;
}
