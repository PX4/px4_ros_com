# px4_ros_com for ROS

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build Status](https://travis-ci.com/PX4/px4_ros_com.svg?token=wyo8gnJf2urtswRL6tUy&branch=ros1)](https://travis-ci.com/PX4/px4_ros_com)

This package materializes the ROS1 side (from now on, referenced as ROS) of PX4-FastRTPS bridge, for interfacing with the ROS2 over [`ros1_bridge`](https://github.com/ros2/ros1_bridge). The [`master`](https://github.com/PX4/px4_ros_com/tree/master) branch of the package repository represents the ROS2 side of the bridge.

The package serves as an example for testing the interface with the ROS2 side, mainly by showing the usage of the [`px4_msgs`](https://github.com/PX4/px4_msgs) `ros1` branch for this same purpose. The idea is that the developer using this package uses the `px4_msgs` on its own ROS nodes so to exchange data with ROS2, and by consequence, with PX4 internals.

The [`ros1`](https://github.com/PX4/px4_ros_com/tree/ros1) branch is a product of the [`master`](https://github.com/PX4/px4_ros_com/tree/master). The interface between the ROS2 and ROS is possible through the [`ros1_bridge`](https://github.com/ros2/ros1_bridge).

## Install, build and usage

Check the [RTPS/ROS2 Interface](https://dev.px4.io/en/middleware/micrortps.html) section on the PX4 Devguide for details on how to install the required dependencies, build the package (composed by the two branches) and use them.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the `#messaging` PX4 Slack channel:
[![Slack](https://px4-slack.herokuapp.com/badge.svg)](http://slack.px4.io)
