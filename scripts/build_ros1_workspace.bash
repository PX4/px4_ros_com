#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros1_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS(1)" >&2
  echo
  echo -e "\t--ros_distro \t\t Set ROS distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros_path \t\t Set ROS environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO"
  echo
  exit 0
fi

SCRIPT_DIR=$(dirname $(realpath -s "$PWD/$0"))

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    if [ ! -z $2 ]; then
      declare $v="$2"
    else
      declare $v=1
    fi
  fi
  shift
done

# One can pass the ROS_DISTRO using the '--ros_distro' arg
unset ROS_DISTRO
if [ -z $ros_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename and
  # ROS install directory
  case "$(lsb_release -s -c)" in
  "xenial")
    if [ -d "/opt/ros/kinetic" ]; then
      ROS_DISTRO="kinetic"
    else
      if [ -z $ros_path ]; then
        echo "- No ROS distro installed or not installed in the default directory."
        echo "  If you are using a ROS version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/kinetic/install). Otherwise, please install ROS Kinetic following http://wiki.ros.org/kinetic/Installation/Ubuntu"
        exit 1
      else
        # source the ROS environment (from arg)
        source $ros_path
      fi
    fi
    ;;
  "bionic")
    if [ -d "/opt/ros/melodic" ]; then
      ROS_DISTRO="melodic"
    else
      if [ -z $ros_path ]; then
        echo "- No ROS distro installed or not installed in the default directory."
        echo "  If you are using a ROS version installed from source, please set the install location with '--ros1_path' arg! (ex: ~/ros_src/melodic/install). Otherwise, please install ROS Melodic following http://wiki.ros.org/melodic/Installation/Ubuntu"
        exit 1
      else
        # source the ROS environment (from arg)
        source $ros_path
      fi
    fi
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
  # source the ROS environment
  source /opt/ros/$ROS_DISTRO/setup.bash
else
  if [ -z $ros_path ]; then
    echo "- Warning: You set a ROS manually to be used."
    echo "  This assumes you want to use another ROS version installed on your system. Please set the install location with '--ros_path' arg! (ex: --ros_path ~/ros_src/eloquent/install/setup.bash)"
    exit 1
  else
    # source the ROS environment (from arg)
    source $ros_path
  fi
fi

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS_PKG_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_PKG_SRC_DIR")" && pwd)

# checks for python-catkin-tools and installs it if not installed
# required to use `catkin_tools`
if [[ "install ok installed" != $(dpkg-query -W --showformat='${Status}\n' python-catkin-tools|grep 'install ok installed') ]]; then
  echo "No python-catkin-tools installed. Installing..."
  sudo apt-get --yes install python-catkin-tools
fi

# build the workspace
printf "\n************* Building ROS1 workspace *************\n\n"
# build the ROS workspace of the px4_ros_com package
cd $ROS_WS_DIR && catkin build

# source the ROS workspace environment so to have it ready to use
source $ROS_WS_DIR/devel/setup.bash

printf "\nROS1 workspace ready...\n\n"
