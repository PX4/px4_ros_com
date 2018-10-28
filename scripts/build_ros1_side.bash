#!/bin/bash
source /opt/ros/$ROS_DISTRO1/setup.bash

# build the ROS1 side of the px4_ros_com package
cd $ROS1_WS_DIR && colcon build --symlink-install --event-handlers console_direct+

# source the workspace
source $ROS1_WS_DIR/install/setup.bash
