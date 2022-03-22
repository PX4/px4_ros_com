import rclpy
from rclpy.node import Node
import numpy as np

from px4_msgs.msg import VehicleOdometry
from px4_interfaces.msg import Trajectory

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trajectory, 'waypoints', 10)
        self.i = 0
        self.waypoints = [[0, 0, -4, 0], [50, 0, -4, 0], [75, 0, -4, 0], [45, 0, -4, 0], [25, 0, -4, 0], [0, 0, -4, 0]]  ## In NED

        timer_period = 1.  # seconds
        self.timer = self.create_timer(timer_period, self.pub_call)
        
        # Placeholders for local position data
        self.local_x = None # north position in NED (meters)
        self.local_y = None # east position in NED (meters)
        self.local_z = None # down position in NED (meters)
        self.local_vx = None # north velocity in NED (metres/sec)
        self.local_vy = None # east velocity in NED (metres/sec)
        self.local_vz = None # down velocity in NED (metres/sec)
        
        self.local_pos_sub_ = self.create_subscription(VehicleOdometry, "fmu/vehicle_odometry/out", self.local_pos_callback, 10)

        self.next_point = self.waypoints[self.i]
        self.current_point = [0, 0, 4, 0]
        self.new_point = True

        self.get_logger().info(" ----Waiting for Local Position Info")

    def pub_call(self):
        msg = Trajectory()  # NED
        msg.x = 1. * self.next_point[0]	#convert to float
        msg.y = 1. * self.next_point[1]
        msg.z = 1. * self.next_point[2]
        msg.yaw = 1. * self.next_point[3]
        self.publisher_.publish(msg)

    def local_pos_callback(self, msg):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z
        self.local_vx = msg.vx
        self.local_vy = msg.vy
        self.local_vz = msg.vz
        self.get_logger().info(" ----Local Position Recieved")
        current_pos = [self.local_x, self.local_y, self.local_z]
        if self.new_point:
            self.new_point = False
            self.get_logger().info(" ----New Way Point Set")
        distance_to_target = np.linalg.norm(current_pos - self.next_point[0:3])
        self.get_logger().info(f" ----Distance from curresnt position to target: {distance_to_target}m")
        if (distance_to_target < 2.0) and (self.i + 1 < len(self.waypoints)):
            self.get_logger().info(" ----POINT REACHED!")
            self.i += 1
            self.current_point = self.next_point
            self.next_point = self.waypoints[self.i]
            self.new_point = True
            

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = MinimalPublisher()

    rclpy.spin(waypoint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
