#!/bin/bash

ROS_DISTRO1=${ROS_DISTRO1:-$(case ${UBUNTU_CODENAME:-"$(lsb_release -s -c)"} in
  "trusty")
    echo "indigo"
    ;;
  "xenial")
    echo "kinetic"
    ;;
  "bouncy")
    echo "melodic"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
esac)}

source /opt/ros/$ROS_DISTRO1/setup.bash

# build the ROS1 side of the px4_ros_com package
ROS1_WS_DIR=${ROS1_WS_DIR:-"$(cd ../../../; pwd)"}
cd $ROS1_WS_DIR && colcon build --symlink-install --event-handlers console_direct+

# source the workspace
source $ROS1_WS_DIR/install/setup.bash
