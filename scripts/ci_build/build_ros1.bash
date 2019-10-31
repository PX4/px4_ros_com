#!/bin/bash
set -e

# source the ROS1 environment
unset ROS_DISTRO
if [ -z $ros1_path ]; then
  source /opt/ros/$ROS1_DISTRO/setup.bash
else
  source $ros1_path
fi

# check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
printf "\n************* Building ROS1 workspace *************\n\n"
# build the ROS1 workspace of the px4_ros_com package
cd $ROS1_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --event-handlers console_direct+

# source the ROS1 workspace environment so to have it ready to use
source $ROS1_WS_DIR/install/setup.bash

printf "\nROS1 workspace ready...\n\n"
exec /bin/bash
