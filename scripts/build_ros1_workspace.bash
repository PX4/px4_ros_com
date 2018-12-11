#!/bin/bash

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros1_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS(1)" >&2
  echo
  echo -e "\t--px4_firmware_dir \t Location of the PX4 Firmware repo. If not set, the FindPX4Firmware CMake module will look for it."
  echo -e "\t--ros_distro \t\t Set ROS distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
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

# One can pass the ROS_DISTRO using the '--ros_distro' arg
if [ -z $ros_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "trusty")
    ROS_DISTRO="indigo"
    ;;
  "xenial")
    ROS_DISTRO="kinetic"
    ;;
  "bionic")
    ROS_DISTRO="melodic"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  ROS_DISTRO="$ros_distro"
fi

SCRIPT_DIR=$PWD

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS_PKG_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_PKG_SRC_DIR")" && pwd)

# Check if the PX4 Firmware dir is passed by argument
PX4_FIRMWARE_DIR=${px4_firmware_dir:-""}

# Source the ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

# build the ROS1 workspace of the px4_ros_com package
cd $ROS_WS_DIR && colcon build --symlink-install --event-handlers console_direct+

# source the workspace
source $ROS_WS_DIR/install/setup.bash
