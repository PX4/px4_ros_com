#ifndef OFFBOARD_EXAMPLE_H
#define OFFBOARD_EXAMPLE_H

#include <control/autopilot_interface.h>
#include <control/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OffboardExample
{
  public:
    explicit OffboardExample(rclcpp::Node& node);
    void arm() const;
    void disarm() const;
    void set_velocity(const control::msg::Twist& twist);

  private:
    OffboardExample();
    void timesync_callback(const px4_msgs::msg::Timesync::UniquePtr msg);
    void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
    void publish_offboard_control_mode() const;
    void publish_position_setpoint_triplet() const;
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) const;
    void px4_offboard_stream();

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
    rclcpp::Publisher<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr position_setpoint_triplet_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber;
    rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr vehicle_gps_position_subscriber;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_subscriber;
    rclcpp::TimerBase::SharedPtr stream_timer;
    control::msg::Twist twist;
    px4_msgs::msg::VehicleControlMode vehicle_control_mode;
    px4_msgs::msg::VehicleGpsPosition vehicle_gps_position;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position;
    px4_msgs::msg::VehicleOdometry vehicle_odometry;
    uint64_t timestamp;
    uint64_t px4_stream_count;
};

#endif
