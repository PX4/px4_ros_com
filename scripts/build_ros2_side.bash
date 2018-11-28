#!/bin/bash

# set the ROS_DISTRO variables
export UBUNTU_CODENAME="$(lsb_release -s -c)"
case "$UBUNTU_CODENAME" in
  "trusty")
    export ROS_DISTRO1="indigo"
    export ROS_DISTRO2="ardent"
    ;;
  "xenial")
    export ROS_DISTRO1="kinetic"
    export ROS_DISTRO2="ardent"
    ;;
  "bionic")
    export ROS_DISTRO1="melodic"
    export ROS_DISTRO2="bouncy"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
esac

SCRIPT_DIR=$PWD
REPO_DIR="$( cd "$(dirname "$SCRIPT_DIR")" && pwd )"
ROS2_PKG_SRC_DIR="$(cd ../../; pwd)"

# by default set to $HOME/px4_ros_com_ros1 but user can use command line arg instead
export ROS1_WS_DIR=${1:-"$( cd "$HOME/px4_ros_com_ros1" && pwd )"}

# source the ROS2 environment
source /opt/ros/$ROS_DISTRO2/setup.bash

ROS2_WS_DIR="$(cd "$(dirname "$ROS2_PKG_SRC_DIR")" && pwd )"

# clone ros1_bridge to the workspace dir
if [ ! -d "$ROS2_PKG_SRC_DIR/ros1_bridge" ]; then
  cd $ROS2_PKG_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git
fi

# build px4_ros_com package, except the ros1_bridge
cd $ROS2_WS_DIR && colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

# source the environments/workspaces so the bridge is be built with support for
# any messages that are on your path and have an associated mapping between ROS 1 and ROS 2
source /opt/ros/$ROS_DISTRO1/setup.bash
source /opt/ros/$ROS_DISTRO2/setup.bash

# check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
if [ -f "$ROS1_WS_DIR/install/setup.bash" ]; then
  source "$ROS1_WS_DIR/install/setup.bash"
else
  source $ROS1_WS_DIR/src/px4_ros_com/scripts/build_ros1_side.bash
fi
source $ROS2_WS_DIR/install/setup.bash

# build the ros1_bridge only
cd $ROS2_WS_DIR && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+
