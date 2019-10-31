#!/bin/bash
set -e

# source the ROS2 workspace environment so to have it ready to use
unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

printf "To start the microRTPS bridge agent, use \"micrortps_agent [options]\"\n\n"
exec /bin/bash
