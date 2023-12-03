#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros2_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS 2" >&2
  echo
  echo -e "\t--ros_distro \t\t Set ROS 2 distro name (dashing|eloquent|foxy|galactic|humble|rolling). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros_path \t\t Set ROS 2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO"
  echo -e "\t--verbose \t\t Add more verbosity to the console output"
  echo
  exit 0
fi

SCRIPT_DIR=$0
if [[ ${SCRIPT_DIR:0:1} != '/' ]]; then
  SCRIPT_DIR=$(dirname $(realpath -s "$PWD/$0"))
fi

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
  # ROS 2 install directory
  # The distros are order by priority (according to being LTS vs non-LTS)
  case "$(lsb_release -s -c)" in
  "bionic")
    if [ -d "/opt/ros/dashing" ]; then
      ROS_DISTRO="dashing"
    elif [ -d "/opt/ros/eloquent" ]; then
      ROS_DISTRO="eloquent"
    else
      if [ -z $ros_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros_path' arg! (ex: ~/ros_src/eloquent/install). Otherwise, please install ROS 2 Dashing following https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros_path
      fi
    fi
    ;;
  "focal")
    if [ -d "/opt/ros/foxy" ]; then
      ROS_DISTRO="foxy"
    elif [ -d "/opt/ros/galactic" ]; then
      ROS_DISTRO="galactic"
    elif [ -d "/opt/ros/rolling" ]; then
      ROS_DISTRO="rolling"
    else
      if [ -z $ros_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros_path' arg! (ex: ~/ros_src/foxy/install). Otherwise, please install ROS 2 Foxy following https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros_path
      fi
    fi
    ;;
  "jammy")
    if [ -d "/opt/ros/humble" ]; then
      ROS_DISTRO="humble"
    elif [ -d "/opt/ros/rolling" ]; then
      ROS_DISTRO="rolling"
    else
      if [ -z $ros_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros_path' arg! (ex: ~/ros_src/foxy/install). Otherwise, please install ROS 2 Humble following https://docs.ros.org/en/humble/Installation.html"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros_path
      fi
    fi
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
  # source the ROS2 environment
  source /opt/ros/$ROS_DISTRO/setup.bash
else
  if [ -z $ros_path ]; then
    echo "- Warning: You set a ROS 2 manually to be used."
    echo "  This assumes you want to use another ROS 2 version installed on your system. Please set the install location with '--ros_path' arg! (ex: --ros_path ~/ros_src/eloquent/install/setup.bash)"
    exit 1
  else
    # source the ROS 2 environment (from arg)
    source $ros_path
  fi
fi

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS_WS_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_WS_SRC_DIR")" && pwd)

# build px4_ros_com package
[ ! -v $verbose ] && colcon_output=$(echo "--event-handlers console_direct+")
cd $ROS_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install $colcon_output

# source the ROS2 workspace environment so to have it ready to use
source $ROS_WS_DIR/install/setup.bash

printf "\nROS2 workspace ready...\n\n"
