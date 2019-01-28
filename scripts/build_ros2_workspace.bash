#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros2_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS2" >&2
  echo
  echo -e "\t--no_ros1_bridge \t Do not clone and build ros1_bridge. Set if only using ROS2 workspace."
  echo -e "\t--ros_distro \t\t Set ROS2 distro name (ardent|bouncy|crystal). If not set, the script will set the ROS2_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros_path \t\t Set ROS2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS2_DISTRO"
  echo
  exit 0
fi

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    declare $v="$2"
  fi
  shift
done

# One can pass the ROS2_DISTRO using the '--ros_distro' arg
if [ -z $ros_distro ]; then
  # set the ROS2_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "xenial")
    export ROS2_DISTRO="bouncy"
    ;;
  "bionic")
    export ROS2_DISTRO="crystal"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
  # source the ROS2 environment
  source /opt/ros/$ROS2_DISTRO/setup.bash
else
  export ROS2_DISTRO="$ros_distro"
  if [ -z $ros_path ]; then
    echo "- Warning: You set a ROS2 distro which is not the supported by default in Ubuntu $(lsb_release -s -c)..."
    echo "           This assumes you want to use another ROS2 version installed on your system. Please set the install location with '--ros_path' arg! (ex: --ros_path ~/ros_src/bouncy/install/setup.bash)"
    exit 1
  else
    # source the ROS2 environment (from arg)
    source $ros_path
  fi
fi

SCRIPT2_DIR=$PWD

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT2_DIR")" && pwd)
ROS_WS_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_WS_SRC_DIR")" && pwd)

# clone ros1_bridge to the workspace dir
if [ -z $no_ros1_bridge ] && [ ! -d "$ROS_WS_SRC_DIR/ros1_bridge" ]; then
  # use $ROS2_DISTRO branch as the latest upstream API changed to fit ROS2 Crystal release
  # if using Crystal otherwise, use master
  ROS1_BRIDGE_RELEASE=$([ $ROS2_DISTRO == "crystal" ] && echo "master" || echo "$ROS2_DISTRO")
  cd $ROS_WS_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS1_BRIDGE_RELEASE
fi

# build px4_ros_com package, except the ros1_bridge
cd $ROS_WS_DIR && colcon build --cmake-args --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

cd $SCRIPT_DIR
