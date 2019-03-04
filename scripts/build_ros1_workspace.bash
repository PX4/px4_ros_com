#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros1_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS(1)" >&2
  echo
  echo -e "\t--ros_distro \t\t Set ROS distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros_path \t\t Set ROS environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO"
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
  # source the ROS environment
  source /opt/ros/$ROS_DISTRO/setup.bash
else
  export ROS_DISTRO="$ros_distro"
  if [ -z $ros_path ]; then
    echo "- Warning: You set a ROS distro which is not the supported by default in Ubuntu $(lsb_release -s -c)..."
    echo "           This assumes you want to use another ROS version installed on your system. Please set the install location with '--ros_path' arg! (ex: --ros_path ~/ros_src/melodic/install/setup.bash)"
    exit 1
  else
    # source the ROS environment (from arg)
    source $ros_path
  fi
fi

SCRIPT_DIR=$PWD

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS_PKG_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_PKG_SRC_DIR")" && pwd)

# check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
printf "\n************* Building ROS1 workspace *************\n\n"
# build the ROS1 workspace of the px4_ros_com package
cd $ROS_WS_DIR && colcon build --symlink-install --event-handlers console_direct+

# source the ROS1 workspace environment so to have it ready to use
source $ROS_WS_DIR/install/local_setup.bash

printf "\nROS1 workspace ready...\n\n"
