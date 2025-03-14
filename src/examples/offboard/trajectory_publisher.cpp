#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher"), counter_(1), mission_count_(0), state_(FLY_LEFT_RIGHT)
    {

        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        // qos_profile.depth = 100;
        // qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 100), qos_profile);

        trajectory_publisher_ = this->create_publisher<TrajectorySetpoint>("/custom_trajectory", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        // subscription_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,std::bind(&TrajectoryPublisher::vehicle_local_position_callback, this, std::placeholders::_1));

        rclcpp::Logger logger = this->get_logger();
        RCLCPP_INFO(logger, "Trajectory Publisher Node Started.");
        // Publishes trajectory every 100ms.
        timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPublisher::publish_trajectory, this));
    }

private:
    enum MissionState { FLY_LEFT_RIGHT, FLY_FORWARD_BACKWARD, FLY_UP_DOWN, ROTATE, DONE };
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
    int mission_count_;
    MissionState state_;
    const int EACH_MOVE_COUNT = 30;
    const int WAITING_TIME = 20;
    /*VehicleLocalPosition current_position_;

    void vehicle_local_position_callback(const VehicleLocalPosition::SharedPtr msg)
    {
        current_position_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current Position - X: %f, Y: %f, Z: %f", 
                current_position_.x, current_position_.y, current_position_.z);

    }*/

    void publish_trajectory()
    {
        TrajectorySetpoint msg{};
        RCLCPP_INFO(this->get_logger(), "Velocity: [%f, %f, %f]", msg.velocity[0], msg.velocity[1], msg.velocity[2]);
        RCLCPP_INFO(this->get_logger(), "Acceleration: [%f, %f, %f]", msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);

        switch (state_)
        {
            case FLY_LEFT_RIGHT:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly left right'-mission.");
                }

                if (counter_ < EACH_MOVE_COUNT) {
                    float progress = static_cast<float>(counter_) / EACH_MOVE_COUNT; // Scale from 0 to 1
                    msg.position = {0.0f, -4.0f * progress, -5.0f}; // Fly left (west)
                } else if ((EACH_MOVE_COUNT <= counter_) && (counter_ < EACH_MOVE_COUNT + WAITING_TIME)) { // Wait the value of "WAITING_TIME" before making new move.
                    msg.position = {0.0f, -4.0f, -5.0f};
                } else if (counter_ < 2 * EACH_MOVE_COUNT + WAITING_TIME) {
                    float progress = static_cast<float>(counter_ - (EACH_MOVE_COUNT + WAITING_TIME)) / EACH_MOVE_COUNT;
                    msg.position = {0.0f, -4.0f + 4.0f * progress, -5.0f};
                } else {
                    msg.position = {0.0f, 0.0f, -5.0f};
                }

                msg.yaw = -3.14;

                if (counter_ >= 2 * EACH_MOVE_COUNT + 2 * WAITING_TIME) {
                    state_ = FLY_FORWARD_BACKWARD;
                    counter_ = 0;
                }

                break;
            
            case FLY_FORWARD_BACKWARD:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly forward backward'-mission.");
                }

                if (counter_ < EACH_MOVE_COUNT) {
                    float progress = static_cast<float>(counter_) / EACH_MOVE_COUNT;
                    msg.position = {4.0f * progress, 0.0f, -5.0f}; // Fly forward (north)
                } else if ((EACH_MOVE_COUNT <= counter_) && (counter_ < EACH_MOVE_COUNT + WAITING_TIME)) { // Wait the value of "WAITING_TIME" before making new move.
                    msg.position = {4.0f, 0.0f, -5.0f};
                } else if (counter_ < 2 * EACH_MOVE_COUNT + WAITING_TIME) {
                    float progress = static_cast<float>(counter_ - (EACH_MOVE_COUNT + WAITING_TIME)) / EACH_MOVE_COUNT;
                    msg.position = {4.0f - (4.0f * progress), 0.0f, -5.0f}; // Go back.
                } else {
                    msg.position = {0.0f, 0.0f, -5.0f}; // Wait in initial state.
                }

                msg.yaw = -3.14;
                
                if (counter_ >= 2 * EACH_MOVE_COUNT + 2 * WAITING_TIME) {
                    state_ = FLY_UP_DOWN;
                    counter_ = 0;
                }

                break;
            
            case FLY_UP_DOWN:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Fly up down'-mission.");
                }
                
                if (counter_ < EACH_MOVE_COUNT) {
                    float progress = static_cast<float>(counter_) / EACH_MOVE_COUNT;
                    msg.position = {0.0f, 0.0f, -5.0f + (2.0f * progress)}; // Go down.
                } else if ((EACH_MOVE_COUNT <= counter_) && (counter_ < EACH_MOVE_COUNT + WAITING_TIME)) { // Wait the value of "WAITING_TIME" before making new move.
                    msg.position = {0.0f, 0.0f, -3.0f};
                } else if (counter_ < 2 * EACH_MOVE_COUNT + WAITING_TIME) {
                    float progress = static_cast<float>(counter_ - (EACH_MOVE_COUNT + WAITING_TIME)) / EACH_MOVE_COUNT;
                    msg.position = {0.0f, 0.0f, -3.0f - (2.0f * progress)};
                } else {
                    msg.position = {0.0f, 0.0f, -5.0f}; // Wait in initial state.
                }

                msg.yaw = -3.14;

                if (counter_ >= 2 * EACH_MOVE_COUNT + 2 * WAITING_TIME) {
                    state_ = ROTATE;
                    counter_ = 0;
                }

                break;

            case ROTATE:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "'Rotate'-mission.");
                }
    
                msg.position = {0.0, 0.0, -5.0}; // Keep position constant
    
                if (counter_ < EACH_MOVE_COUNT) {
                    // Gradually increase yaw from -π to π (full rotation counterclockwise)
                    msg.yaw = -M_PI + (2 * M_PI * counter_ / EACH_MOVE_COUNT);
                } else if ((EACH_MOVE_COUNT <= counter_) && (counter_ < EACH_MOVE_COUNT + WAITING_TIME)) { 
                    // Wait at π for "WAITING_TIME"
                    msg.yaw = M_PI;
                } else if (counter_ < 2 * EACH_MOVE_COUNT + WAITING_TIME) {
                    // Gradually decrease yaw from π to -π (full rotation clockwise)
                    msg.yaw = M_PI - (2 * M_PI * (counter_ - EACH_MOVE_COUNT - WAITING_TIME) / EACH_MOVE_COUNT);
                } else if ((2 * EACH_MOVE_COUNT + WAITING_TIME <= counter_) && (counter_ < 2 * EACH_MOVE_COUNT + 2 * WAITING_TIME)) {
                    // Wait at -π for "WAITING_TIME" before repeating
                    msg.yaw = -M_PI;
                } else if (counter_ >= 2 * EACH_MOVE_COUNT + 2 * WAITING_TIME) {
                    msg.yaw = -M_PI;
                    mission_count_++;
    
                    if (mission_count_ < 3) {
                        state_ = FLY_LEFT_RIGHT;
                    } else {
                        state_ = DONE; // Stop sending commands
                    }
    
                    counter_ = 0;
                }
    
                break;
            
            case DONE:
                if (counter_ == 1) {
                    RCLCPP_INFO(this->get_logger(), "Mission done. Set LAND-mode.");
                }

                set_land_mode();

                return;
        }
        
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        counter_++;
        trajectory_publisher_->publish(msg);
    }

    void set_land_mode();
};

// TODO: This function should be before deploying it.
void TrajectoryPublisher::set_land_mode()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE; // Command to change mode
    msg.param1 = 1;  // Custom mode
    msg.param2 = 6;  // 6 corresponds to LAND mode in PX4
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    vehicle_command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "LAND mode set.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
