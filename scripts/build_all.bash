#!/bin/bash

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_all.bash [option...] \t This scripts builds px4_ros_com workspaces for ROS2 and ROS(1)" >&2
  echo
  echo -e "\t--px4_firmware_dir \t Location of the PX4 Firmware repo. If not set, the FindPX4Firmware CMake module will look for it."
  echo -e "\t--ros1_ws_dir \t\t Location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t\t Set ROS1 distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu version name"
  echo -e "\t--ros2_distro \t\t Set ROS2 distro name (ardent|bouncy). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu version name"
  echo
  return 0
fi

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    declare $v="$2"
  fi
  shift
done

# One can pass the ROS_DISTRO's using the '--ros1_distro' and '--ros2_distro' args
if [ -z $ros1_distro ] && [ -z $ros2_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu version
  export UBUNTU_CODENAME="$(lsb_release -s -c)"
  case "$UBUNTU_CODENAME" in
  "trusty")
    export ROS1_DISTRO="indigo"
    export ROS2_DISTRO="ardent"
    ;;
  "xenial")
    export ROS1_DISTRO="kinetic"
    export ROS2_DISTRO="ardent"
    ;;
  "bionic")
    export ROS1_DISTRO="melodic"
    export ROS2_DISTRO="bouncy"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  export ROS1_DISTRO="$ros1_distro"
  export ROS2_DISTRO="$ros2_distro"
fi

SCRIPT_DIR=$PWD

# setup the required path variables
ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS2_PKG_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
ROS2_WS_DIR=$(cd "$(dirname "$ROS2_PKG_SRC_DIR")" && pwd)

# by default set to $HOME/px4_ros_com_ros1 but user can use command line arg instead
export ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# source the ROS2 environment
source /opt/ros/$ROS2_DISTRO/setup.bash

# clone ros1_bridge to the workspace dir
if [ ! -d "$ROS2_PKG_SRC_DIR/ros1_bridge" ]; then
  # use $ROS2_DISTRO branch as the latest upstream API changed to fit ROS2 Crystal release
  cd $ROS2_PKG_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS2_DISTRO
fi

# Check if the PX4 Firmware dir is passed by argument
PX4_FIRMWARE_DIR=${px4_firmware_dir:-""}

# build px4_ros_com package, except the ros1_bridge
cd $ROS2_WS_DIR && colcon build --cmake-args -DPX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

# source the environments/workspaces so the bridge is be built with support for
# any messages that are on your path and have an associated mapping between ROS 1 and ROS 2
source /opt/ros/$ROS1_DISTRO/setup.bash
source /opt/ros/$ROS2_DISTRO/setup.bash

# check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
if [ -f "$ROS1_WS_DIR/install/setup.bash" ]; then
  source "$ROS1_WS_DIR/install/setup.bash"
else
  source $ROS1_WS_DIR/src/px4_ros_com/scripts/build_ros1_side.bash
fi
source $ROS2_WS_DIR/install/setup.bash

# build the ros1_bridge only
cd $ROS2_WS_DIR && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+

cd $SCRIPT_DIR
