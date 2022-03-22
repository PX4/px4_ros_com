import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleGpsPosition


from px4_interfaces.msg import Trajectory


class OffboardControl(Node):

    def __init__(self, pos=[0., 0., -4., 0.]):
        super().__init__('OffboardControl')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                      "fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/trajectory_setpoint/in",
                                                                    10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/vehicle_command/in", 10)

        # Get common timestamp
        self.timestamp_ = 0
        self.timesync_sub_ = self.create_subscription(VehicleCommand, "fmu/vehicle_command/in", self.timesync_callback, 10)

        self.offboard_setpoint_counter_ = 0

        # Initial values till it recieves waypoints
        self.pos_x = pos[0]
        self.pos_y = pos[1]
        self.pos_z = pos[2]
        self.pos_yaw = pos[3]
        
        self.waypoint_sub_ = self.create_subscription(Trajectory, "waypoints", self.waypoint_callback, 10)

        timer_period = 0.1  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        
        # Initial values for gps data
        self.lat = None	# latitude in degrees
        self.lon = None	# longitude in degrees
        self.alt = None	# altitude in above MSL
        self.vel_m_s = None	# GPS ground speed, (metres/sec) 
        
        self.gps_sub_ = self.create_subscription(VehicleGpsPosition, "fmu/vehicle_gps_position/out", self.gps_callback, 10)
        

    def timesync_callback(self, msg):
        self.timestamp_ = msg.timestamp

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    '''
	Publish vehicle commands
	command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
	param1    Command parameter 1
                VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
	            VEHICLE_MODE_FLAG_TEST_ENABLED = 2, /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
	            VEHICLE_MODE_FLAG_AUTO_ENABLED = 4, /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
	            VEHICLE_MODE_FLAG_GUIDED_ENABLED = 8, /* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
	            VEHICLE_MODE_FLAG_STABILIZE_ENABLED = 16, /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
	            VEHICLE_MODE_FLAG_HIL_ENABLED = 32, /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
	            VEHICLE_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, /* 0b01000000 remote control input is enabled. | */

	param2    Command parameter 2
                PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
	            PX4_CUSTOM_MAIN_MODE_ALTCTL = 2,
	            PX4_CUSTOM_MAIN_MODE_POSCTL = 3,
	            PX4_CUSTOM_MAIN_MODE_AUTO = 4,
	            PX4_CUSTOM_MAIN_MODE_ACRO = 5,
	            PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6,
	            PX4_CUSTOM_MAIN_MODE_STABILIZED = 7,
	            PX4_CUSTOM_MAIN_MODE_RATTITUDE_LEGACY = 8
    '''

    def publish_vehicle_command(self, command, param1=0.0,
                                param2=0.0):  # parameter 1 and 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

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
        msg.x = self.pos_x
        msg.y = self.pos_y
        msg.z = self.pos_z
        msg.yaw = self.pos_yaw;  # [-PI:PI]

        self.trajectory_setpoint_publisher_.publish(msg)
        
    
    def gps_callback(self, msg):
        self.get_logger().info("RECEIVED VEHICLE GPS POSITION DATA")
        self.get_logger().info(f"lat: {msg.lat/1E7}")
        self.get_logger().info(f"lon: {msg.lon/1E7}")
        self.get_logger().info(f"alt: {msg.alt/1E3}")
        self.get_logger().info(f"alt_el: {msg.alt_ellipsoid/1E3}")
        self.get_logger().info(f"vel_m_s: {msg.vel_m_s}")
        self.lat = msg.lat/1E7	# msg.latitude in 1E-7 degrees
        self.lon = msg.lon/1E7	# msg.longitude in 1E-7 degrees
        self.alt = msg.alt/1E3	# msg.altitude in 1E-3 meters above MSL
        self.vel_m_s = msg.vel_m_s	# GPS ground speed, (metres/sec)
        
    
    def waypoint_callback(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z
        self.pos_yaw = msg.yaw
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
