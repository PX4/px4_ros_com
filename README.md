# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build and Test package](https://github.com/PX4/px4_ros_com/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/PX4/px4_ros_com/actions)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This package provides example nodes for exchanging data and commands between ROS2 and PX4.
It also provides a [library](./include/px4_ros_com/frame_transforms.h) to ease the conversion between ROS2 and PX4 frame conventions.
It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package.

## Install, build and usage

Check the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) and the [ROS2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html) sections on the PX4 Devguide for details on how to install the required dependencies, build the package and use it.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the [PX4 Discord Server](https://discord.gg/dronecode).


## PREREQ: Run XRCE bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

## PREREQ: Run Gazebo simulation
```bash
cd xxx/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500 # headless if you don't have gui
```

# Custom stuff for CatScanners

## PREREQ: Build PX4_ros_com package (might have to source ROS first)
```bash
colcon build
```

## PREREQ: Source setup files
```bash
source install/setup.bash
```

## Run flight example
First, start to track telemetry to verify results later.
```bash
ros2 run px4_ros_com vehicle_gps_position_listener
```

Disable the safe mode.
```bash
mavproxy.py --master=udp:127.0.0.1:14540
param set COM_FLTMODE2 7 # 7 = Offboard, as mentioned here https://docs.px4.io/main/en/advanced_config/parameter_reference.html#commander
```

Offboard control node (by default goes to 6m).
```bash
ros2 run px4_ros_com offboard_control
```

Run offboard commands on demand:
```bash
ros2 topic pub /custom_trajectory px4_msgs/msg/TrajectorySetpoint "{ position: [ 0.0, 0.0, -50.0 ], velocity: [0.0, 0.0, 0.0],  yaw: -3.14 }"
```

## Run predefined motion (up, left, forward, rotate) x2:
1. Start offboard node
```
ros2 run px4_ros_com offboard_control # start offboard node
```
2. Arm from QGroundCtonrol / via mavproxy by running ```arm throttle```.
3. Record
```
ros2 bag record /fmu/out/vehicle_local_position
```
4. Start motions
```
ros2 run px4_ros_com maneuver.py # start the script
```
