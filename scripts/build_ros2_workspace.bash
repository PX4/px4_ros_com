#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_ros2_workspace.bash [option...] \t This script builds px4_ros_com workspace for ROS2" >&2
  echo
  echo -e "\t--no_ros1_bridge \t Do not clone and build ros1_bridge. Set if only using ROS2 workspace."
  echo -e "\t--ros_distro \t\t Set ROS2 distro name (ardent|bouncy|crystal|dashing). If not set, the script will set the ROS2_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros_path \t\t Set ROS2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS2_DISTRO"
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

# One can pass the ROS2_DISTRO using the '--ros_distro' arg
unset ROS_DISTRO
if [ -z $ros_distro ]; then
  # set the ROS2_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "xenial")
    export ROS2_DISTRO="ardent"
    ;;
  "bionic")
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

# workaround for rosidl fix in Dashing, while backport isn't applied from Eloquent
# source ~/PX4/rosidl_ws/install/setup.bash

# setup the required path variables
ROS_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS_WS_SRC_DIR=$(cd "$(dirname "$ROS_REPO_DIR")" && pwd)
ROS_WS_DIR=$(cd "$(dirname "$ROS_WS_SRC_DIR")" && pwd)

# clone ros1_bridge to the workspace dir
if [ -z $no_ros1_bridge ] && [ ! -d "$ROS_WS_SRC_DIR/ros1_bridge" ]; then
  cd $ROS_WS_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS2_DISTRO
fi

# build px4_ros_com package, except the ros1_bridge
cd $ROS_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+

# source the ROS2 workspace environment so to have it ready to use
unset ROS_DISTRO && source $ROS_WS_DIR/install/setup.bash

printf "\nROS2 workspace ready...\n\n"
