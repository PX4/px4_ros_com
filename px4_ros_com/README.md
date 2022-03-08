# PX4-ROS2 bridge

This package materializes the ROS2 side of PX4-FastRTPS/DDS bridge, establishing a bridge between the PX4 autopilot stack through a micro-RTPS bridge, Fast-RTPS(DDS) and ROS2. It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package, as it depends on the IDL files, to generate the micro-RTPS bridge agent, and on the ROS interfaces and typesupport, to allow building and running the example nodes.

The [`master`](https://github.com/PX4/px4_ros_com/tree/master) branch of this package composes the ROS2 package and the ROS2 side (agent) of the bridge. The [`ros1`](https://github.com/PX4/px4_ros_com/tree/ros1) branch is a product of the `master` and represents the ROS(1) package and the ROS(1) side of the bridge, for wich it is required using the [`ros1_bridge`](https://github.com/ros2/ros1_bridge).


## Install

On a clean setup run this

```
source /opt/ros/foxy/setup.bash
mkdir -p colcon-ws/src && cd colcon-ws/src
git clone git@github.com:PX4/px4_msgs.git
git clone git@github.com:PX4/px4_ros_com.git
cd ..
colcon build
```


## Run the bridge

```
source install/setup.bash
micrortps_agent -t UDP
```