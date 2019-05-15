#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_all.bash [option...] \t This script builds px4_ros_com workspaces for ROS2 and ROS(1)" >&2
  echo
  echo -e "\t--ros1_ws_dir \t\t Location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
  echo -e "\t--ros1_distro \t\t Set ROS1 distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t\t Set ROS2 distro name (ardent|bouncy|crystal). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros1_path \t\t Set ROS(1) environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS1_DISTRO/"
  echo -e "\t--ros2_path \t\t Set ROS2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS2_DISTRO/"
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

# One can pass the ROS_DISTRO's using the '--ros1_distro' and '--ros2_distro' args
unset ROS_DISTRO
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
  # source the ROS1 environment (required so px4_ros_com uses genmsg)
  source /opt/ros/$ROS1_DISTRO/setup.bash
else
  export ROS1_DISTRO="$ros1_distro"
  if [ -z $ros1_path ]; then
    echo "- Warning: You set a ROS(1) distro which is not the supported by default in Ubuntu $(lsb_release -s -c)..."
    echo "           This assumes you are using a ROS version installed from source. Please set the install location with '--ros_path' arg! (ex: ~/ros_src/kinetic/devel)"
    exit 1
  else
    # source the ROS1 environment (from arg)
    source $ros1_path
  fi
fi
if [ -z $ros2_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
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
export ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
export ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# by default set to $HOME/px4_ros_com_ros1 but user can use command line arg instead
export ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}

# clone ros1_bridge to the workspace dir
if [ -z $no_ros1_bridge ] && [ ! -d "$ROS2_WS_SRC_DIR/ros1_bridge" ]; then
  cd $ROS2_WS_SRC_DIR && git clone https://github.com/ros2/ros1_bridge.git -b $ROS2_DISTRO
fi

gnome-terminal --tab -- /bin/bash -c \
  '''
    # source the ROS1 environment
    unset ROS_DISTRO
    if [ -z $ros1_path ]; then
      source /opt/ros/$ROS1_DISTRO/setup.bash
    else
      source $ros1_path
    fi

    # check if the ROS1 side of px4_ros_com was built and source it. Otherwise, build it
    printf "\n************* Building ROS1 workspace *************\n\n"
    # build the ROS1 workspace of the px4_ros_com package
    cd $ROS1_WS_DIR && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELWITHDEBINFO --symlink-install --event-handlers console_direct+

    # source the ROS1 workspace environment so to have it ready to use
    source $ROS1_WS_DIR/install/setup.bash

    printf "\nROS1 workspace ready...\n\n"
    exec /bin/bash
  '''

printf "\n************* Building ROS2 workspace *************\n\n"
# build px4_ros_com package, except the ros1_bridge
cd $ROS2_WS_DIR && colcon build --packages-skip ros1_bridge --event-handlers console_direct+

gnome-terminal --tab -- /bin/bash -c \
  '''
    unset ROS_DISTRO
    # check if the ROS1 workspace of px4_ros_com and px4_msgs was built and source it.
    if [ -z $ROS1_WS_DIR ]; then
      echo "ROS1 workspace does not exist!"
      exec /bin/bash
      exit 1
    else
      if [ -f $ROS1_WS_DIR/install/setup.bash ]; then
        unset ROS_DISTRO && source $ROS1_WS_DIR/install/setup.bash
      else
        echo "ROS1 workspace not built."
        exec /bin/bash
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
    exec /bin/bash
  '''

# source the ROS2 workspace environment so to have it ready to use
unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

printf "\nROS2 workspace ready...\n\n"

gnome-terminal --tab -- /bin/bash -c \
  '''
  # source the ROS2 workspace environment so to have it ready to use
  unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

  printf "To start the microRTPS bridge agent, use \"micrortps_agent [options]\"\n\n"
  exec /bin/bash
  '''
