# px4_roscom

This package materializes the PX4-FastRTPS bridge, with an interface between the PX4 autopilot stack, ROS2 and ROS1 (from now on, referenced as ROS) over [`ros1_bridge`](https://github.com/ros2/ros1_bridge).

The package is divided into two different branches:

1. The [`master`](https://github.com/PX4/px4_ros_com/tree/master) branch composes the required Cmake modules and build structure to generate the ROS2 message headers and source files, which are then used and propagated on the RTPS DDS layer;
2. The [`ros1`](https://github.com/PX4/px4_ros_com/tree/ros1) branch is a product of the `master` branch and its purpose is also generate the ROS message headers and source files, but to use on the ROS side. The interface between the ROS2 and ROS is possible through the [`ros1_bridge`](https://github.com/ros2/ros1_bridge).
