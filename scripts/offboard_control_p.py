#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import Cpuload

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.timestamp_ = 0
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, 'OffboardControlMode_PubSubTopic', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, 'TrajectorySetpoint_PubSubTopic', 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, 'VehicleCommand_PubSubTopic', 10)  
        self.offboard_setpoint_counter_ = 0

        def timesync_callback(msg):
            self.timestamp_ = msg.timestamp

        def timer_callback():
            if self.offboard_setpoint_counter_ == 10:
				# Change to Offboard mode after 10 setpoints
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
				# Arm the vehicle
                self.arm()

            # Offboard_control_mode needs to be paired with trajectory_setpoint
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()


           	# Stop the counter after reaching 11
            if self.offboard_setpoint_counter_ < 11:
                self.offboard_setpoint_counter_ += 1
        
        self.timesync_sub_ = self.create_subscription(Timesync, 'Timesync_PubSubTopic', timesync_callback, 10)
        print('create_subscription Timesync_PubSubTopic')
        self.log_ = self.get_logger()
        self.timer_ = self.create_timer(0.1, timer_callback)
        print('create_timer')


       
    '''
    Publish the offboard control mode.
    For this example, only position and altitude controls are active.
    '''
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_control_mode_publisher_.publish(msg)

    '''
    Publish a trajectory setpoint
    For this example, it sends a trajectory setpoint to make the
    vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''
    def publish_trajectory_setpoint(self):   
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_
        msg.x = 0.0
        msg.y = 0.0
        msg.z = -5.0
        msg.yaw = -3.14
        
        self.trajectory_setpoint_publisher_.publish(msg)

    '''
    Publish vehicle commands
    command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
    param1    Command parameter 1
    param2    Command parameter 2
    '''
    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher_.publish(msg)
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        self.log_.info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        self.log_.info("Disarm command send")

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()