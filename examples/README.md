# Examples

This directory contains a set of examples application for the PX4-ROS2 bridge. All examples are intended to demonstrate basic interactions between PX4 and ROS2 by sending or receiving [px4_msgs](https://github.com/PX4/px4_msgs).
Feel free to play with the code, extend it and copy it into your own project.

- We assume that your have a basic understanding of running [ROS2](https://docs.ros.org/) applications as well as [PX4](https://docs.px4.io/master/en/).

- Also make sure you have build the [px4_ros_com](../px4_ros_com#install) package successfully.

## Layout

There are several separate examples packages. Each example package assembles its own executables, where each executable can be used in a standalone fashion to interact with a PX4 vehicle from your ROS2 environment.

The examples are structured (by package and executable):

* [listeners](./listeners): a set of short examples to read data from PX4 in a ROS2 application
   
    - sensor_combined_listener
    - vehicle_odometry_listener

* [publishers](./publishers): a set of short examples publishing messages from ROS2 towards PX4
   
    - debug_vect_publisher
    - arm_publisher
    - disarm_publisher
    - takeoff_publisher
    - land_publisher

* [offboard](./offboard): a more complex example that arms a robots and sends it to a defined position
   
    - offboard_control


## Using Examples

All examples are designed to work with a most possible variety of robots. This can be a multirotor, fixed-wing or VTOL flying robot, a rover or boat. While the examples work on most robots right-away, the are designed to wok in simulation. Maybe you need to adapt the bridge configuration or topic names.

Usually an example needs two things

1. The simulated robot. E.g.:

   ```
   user@ubuntu:~/PX4-Autopilot$ make px4_sitl_rtps gazebo_iris
   ```

2. The microRTPS agent:

   ```
   user@ubuntu:~/colcon-ws$ micrortps_agent -t UDP
   ```

Detailed instructions and specific limitations will be in the README file for each package or in the section related to a specific binary in the package.

Read about more robots and simulated worlds [here](https://docs.px4.io/master/en/simulation/gazebo.html).

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).
