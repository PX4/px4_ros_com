# Listener Examples

This package contains two executables that subscribe to PX4 robot data and print it to the screen. 


For both examples start any robot, e.g.

```
user@ubuntu:~/PX4-Autopilot$ make px4_sitl_rtps gazebo_boat
```
 
in one terminal and in a new terminal the bridge agent

```
user@ubuntu:~/colcon-ws$ micrortps_agent -t UDP
```

## sensor_combined_listener

The `sensor_combined_listener` binary subscribes to the ROS2 message [SensorCombined](https://github.com/PX4/px4_msgs/blob/master/msg/SensorCombined.msg) under the topic `fmu/sensor_combined/out`. You can find the related uORB message definition on the PX4 repository under [sensor_combined](https://github.com/PX4/PX4-Autopilot/blob/master/msg/sensor_combined.msg).

This message gives you the accelerometer and gyrometer data of your PX4 robot.

Run 

```
user@ubuntu:~/colcon-ws$ ros2 run px4_ros_com_listener_examples sensor_combined_listener
```

You should see something like this:

```
user@ubuntu:~/colcon-ws$ ros2 run px4_ros_com_listener_examples sensor_combined_listener 
Starting sensor_combined listener node...


RECEIVED SENSOR COMBINED DATA
=============================
ts: 1634661989636389
gyro_rad[0]: 0.00133158
gyro_rad[1]: 0.0015979
gyro_rad[2]: 0.000798948
gyro_integral_dt: 4000
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.00957681
accelerometer_m_s2[1]: -0.0814027
accelerometer_m_s2[2]: -9.65941
accelerometer_integral_dt: 4000
```

If you start another terminal, you can see the same data with

```
user@ubuntu:~/colcon-ws$ ros2 topic echo fmu/sensor_combined/out
```


## vehicle_odometry_listener

The `vehicle_odometry_listener` binary subscribes to the ROS2 message [VehicleOdometry](https://github.com/PX4/px4_msgs/blob/master/msg/VehicleOdometry.msg) under the topic `fmu/sensor_combined/out`. You can find the related uORB message definition on the PX4 repository under [vehicle_odometry](https://github.com/PX4/PX4-Autopilot/blob/master/msg/vehicle_odometry.msg).

This message gives you the local position, orientation, velocities and their covariances.

Run 

```
user@ubuntu:~/colcon-ws$ ros2 run px4_ros_com_listener_examples vehicle_odometry_listener
```

You should see something like this:

```
user@ubuntu:~/colcon-ws$ ros2 run px4_ros_com_listener_examples vehicle_odometry_listener 
Starting vehicle_odometry listener node...


RECEIVED VEHICLE ODOMETRY DATA
==================================
timestamp: 1634662773746560
position: -0.00876204, -0.00276313, -0.0202104
velocity: -0.0124731, -0.0161834, 0.0285649
orientation: 0.709173, -0.00153102, 0.003132, 0.705025
angular vel: 0.00109032, -0.000831208, 0.000450864
```

If you start another terminal, you can see the same data with

```
user@ubuntu:~/colcon-ws$ ros2 ros2 topic echo fmu/vehicle_odometry/out
```

