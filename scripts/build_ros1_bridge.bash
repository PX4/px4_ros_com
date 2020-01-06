#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros1_bridge.bash [option...] \t This script build the ros1_bridge package." >&2
  echo
  echo "NOTE: The script requires both ROS2 and ROS(1) workspaces to be built previously." >&2
  echo
  echo -e "\t--ros1_ws_dir \t Location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t Set ROS1 distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t Set ROS2 distro name (ardent|bouncy|crystal|dashing). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo
  exit 0
fi

SCRIPT_DIR=$(dirname $(realpath -s "$PWD/$0"))

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    declare $v="$2"
  fi
  shift
done

# Get FastRTPSGen version
fastrtpsgen_version_out=""
if [[ -z $FASTRTPSGEN_DIR ]]; then
  fastrtpsgen_version_out="$FASTRTPSGEN_DIR/$(fastrtpsgen -version)"
else
  fastrtpsgen_version_out=$(fastrtpsgen -version)
fi
if [[ -z $fastrtpsgen_version_out ]]; then
  echo "FastRTPSGen not found! Please build and install FastRTPSGen..."
  exit 1
else
  fastrtpsgen_version="${fastrtpsgen_version_out: -5:-2}"
  if ! [[ $fastrtpsgen_version =~ ^[0-9]+([.][0-9]+)?$ ]] ; then
    fastrtpsgen_version="1.0"
  fi
  echo "FastRTPSGen version major: ${fastrtpsgen_version}"
fi

# One can pass the ROS_DISTRO's using the '--ros1_distro' and '--ros2_distro' args
if [ -z $ros1_distro ] && [ -z $ros2_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "xenial")
    ROS1_DISTRO="kinetic"
    ROS2_DISTRO="ardent"
    ;;
  "bionic")
    ROS1_DISTRO="melodic"
    if [[ $fastrtpsgen_version == "1.5" || $fastrtpsgen_version == "1.6" ]]; then
      ROS2_DISTRO="bouncy"
    elif [[ $fastrtpsgen_version == "1.7" ]]; then
      ROS2_DISTRO="crystal"
    else
      ROS2_DISTRO="dashing"
    fi
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  ROS1_DISTRO="$ros1_distro"
  ROS2_DISTRO="$ros2_distro"
fi

# ROS2 dirs
ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# ROS1 dirs (one can pass the ROS1 workspace dir using '--ros1_ws_dir <ws_dir>')
ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# check if the ROS1 workspace of px4_ros_com was built and source it.
if [ -z $ROS1_WS_DIR ]; then
  echo "ROS1 workspace does not exist!"
  exit 1
else
  if [ -f $ROS1_WS_DIR/install/setup.bash ]; then
    unset ROS_DISTRO && source $ROS1_WS_DIR/install/setup.bash
  else
    echo "ROS1 workspace not built."
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

cd $SCRIPT_DIR
