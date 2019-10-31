#!/bin/bash
set -e

unset ROS_DISTRO
# check if the ROS1 workspace of px4_ros_com and px4_msgs was built and source it.
if [ -z $ROS1_WS_DIR ]; then
  echo "ROS1 workspace does not exist!"
  exec /bin/bash
  exit 1
else
  if [ -f $ROS1_WS_DIR/install/setup.bash ]; then
    unset ROS_DISTRO && source $ROS1_WS_DIR/install/setup.bash
  else
    echo "ROS1 workspace not built."
    exec /bin/bash
    exit 1
  fi
fi

# source the ROS2 workspace
unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

printf "\n************* Building ros1_bridge *************\n\n"
# build the ros1_bridge only
cd $ROS2_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+

# source the ROS2 workspace environment so to have it ready to use
unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

printf "\nros1_bridge workspace ready...\n\n"
exec /bin/bash
