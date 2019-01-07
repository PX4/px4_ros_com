#!/bin/bash

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_all.bash [option...] \t This script builds px4_ros_com workspaces for ROS2 and ROS(1)" >&2
  echo
  echo -e "\t--px4_firmware_dir \t Location of the PX4 Firmware repo. If not set, the FindPX4Firmware CMake module will look for it."
  echo -e "\t--ros1_ws_dir \t\t Location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t\t Set ROS1 distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t\t Set ROS2 distro name (ardent|bouncy). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros1_path \t\t Set ROS(1) environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS1_DISTRO/"
  echo -e "\t--ros2_path \t\t Set ROS2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS2_DISTRO/"
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
if [ -z $ros1_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "trusty")
    export ROS1_DISTRO="indigo"
    ;;
  "xenial")
    export ROS1_DISTRO="kinetic"
    ;;
  "bionic")
    export ROS1_DISTRO="melodic"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  export ROS1_DISTRO="$ros1_distro"
  if [ -z $ros1_path ]; then
    echo "- Warning: You set a ROS(1) distro which is not the supported by default in Ubuntu $(lsb_release -s -c)..."
    echo "           This assumes you are using a ROS version installed from source. Please set the install location with '--ros_path' arg! (ex: ~/ros_src/kinetic/devel)"
    exit 1
  fi
fi
if [ -z $ros2_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "trusty")
    export ROS2_DISTRO="ardent"
    ;;
  "xenial")
    export ROS2_DISTRO="ardent"
    ;;
  "bionic")
    export ROS2_DISTRO="bouncy"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  export ROS2_DISTRO="$ros2_distro"
  if [ -z $ros2_path ]; then
    echo "- Warning: You set a ROS2 distro which is not the supported by default in Ubuntu $(lsb_release -s -c)..."
    echo "           This assumes you are using a ROS2 version installed from source. Please set the install location with '--ros_path' arg! (ex: ~/ros_src/bouncy/install)"
    exit 1
  else
    # source the ROS2 environment (from arg)
    source $ros2_path
  fi
fi

SCRIPT_DIR=$PWD

# setup the required path variables
export ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
export ROS2_PKG_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
export ROS2_WS_DIR=$(cd "$(dirname "$ROS2_PKG_SRC_DIR")" && pwd)

# by default set to $HOME/px4_ros_com_ros1 but user can use command line arg instead
export ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# clone ros1_bridge to the workspace dir
if [ ! -d "$ROS2_PKG_SRC_DIR/ros1_bridge" ]; then
  # use $ROS2_DISTRO branch as the latest upstream API changed to fit ROS2 Crystal release
  cd $ROS2_PKG_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS2_DISTRO
fi

# Check if the PX4 Firmware dir is passed by argument
PX4_FIRMWARE_DIR=${px4_firmware_dir:-""}

# build px4_ros_com package, except the ros1_bridge
cd $ROS2_WS_DIR && colcon build --cmake-args -DPX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

# check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
if [ -f "$ROS1_WS_DIR/install/setup.bash" ]; then
  source "$ROS1_WS_DIR/install/setup.bash"
else
  # source the ROS1 environment
  if [ -z $ros1_path ]; then
    source /opt/ros/$ROS1_DISTRO/setup.bash
  else
    source $ros1_path
  fi

  # build the ROS1 workspace of the px4_ros_com package
  cd $ROS1_WS_DIR && colcon build --cmake-args -DPX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR --symlink-install --event-handlers console_direct+
fi

# source the environments/workspaces so the bridge is be built with support for
# any messages that are on your path and have an associated mapping between ROS 1 and ROS 2
if [ -z $ros1_path ]; then
  source /opt/ros/$ROS1_DISTRO/setup.bash
else
  source $ros1_path
fi
if [ -z $ros1_path ]; then
  source /opt/ros/$ROS2_DISTRO/setup.bash
else
  source $ros2_path
fi

# source the ROS workspaces
source $ROS2_WS_DIR/install/setup.bash
source $ROS1_WS_DIR/install/setup.bash

# build the ros1_bridge only
cd $ROS2_WS_DIR && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+

cd $SCRIPT_DIR
