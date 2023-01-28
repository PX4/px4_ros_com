#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: build_all.bash [option...] \t This script builds px4_ros_com workspaces for ROS 2 and ROS (1)" >&2
  echo
  echo -e "\t--ros2_distro \t\t Set ROS 2 distro name (dashing|eloquent|foxy|galactic|humble|rolling). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_path \t\t Set ROS 2 environment setup.bash location. Useful for source installs. If not set, the script sources the environment in /opt/ros/$ROS_DISTRO/"
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

unset ROS_DISTRO
# One can pass the ROS_DISTRO's using the '--ros2_distro' args
if [ -z $ros2_distro]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename and
  # ROS install directory
  # The distros are order by priority (according to being LTS vs non-LTS)
  case "$(lsb_release -s -c)" in "bionic")

    if [ -d "/opt/ros/foxy" ]; then
      export ROS2_DISTRO="foxy"
    elif [ -d "/opt/ros/galactic" ]; then
      export ROS2_DISTRO="galactic"             
    elif [ -d "/opt/ros/humble" ]; then
      export ROS2_DISTRO="humble"            

    else
      if [ -z $ros2_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros2_path' arg! (ex: ~/ros_src/eloquent/install). Otherwise, please install ROS 2 Dashing following https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros2_path
        export ROS2_DISTRO="$(rosversion -d)"
      fi
    fi
    ;;
  "focal")

    if [ -d "/opt/ros/foxy" ]; then
      export ROS2_DISTRO="foxy"
    elif [ -d "/opt/ros/galactic" ]; then
      ROS_DISTRO="galactic"
    elif [ -d "/opt/ros/rolling" ]; then
      ROS_DISTRO="rolling"
    else
      if [ -z $ros2_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros2_path' arg! (ex: ~/ros_src/foxy/install). Otherwise, please install ROS 2 Foxy following https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Binary/"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros2_path
        export ROS2_DISTRO="$(rosversion -d)"
      fi
    fi
    ;;
  "jammy")

    if [ -d "/opt/ros/humble" ]; then
      ROS2_DISTRO="humble"
    elif [ -d "/opt/ros/rolling" ]; then
      ROS2_DISTRO="rolling"
    else
      if [ -z $ros2_path ]; then
        echo "- No ROS 2 distro installed or not installed in the default directory."
        echo "  If you are using a ROS 2 version installed from source, please set the install location with '--ros2_path' arg! (ex: ~/ros_src/humble/install). Otherwise, please install ROS 2 Humble following https://docs.ros.org/en/humble/Installation.html"
        exit 1
      else
        # source the ROS2 environment (from arg)
        source $ros2_path
        export ROS2_DISTRO="$(rosversion -d)"
      fi
    fi
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else

  if [ -z $ros2_path ]; then
    echo "- Warning: You set a ROS 2 manually to be used."
    echo "  This assumes you want to use another ROS 2 version installed on your system. Please set the install location with '--ros2_path' arg! (ex: --ros_path ~/ros_src/eloquent/install/setup.bash)"
    exit 1
  else
    export ROS2_DISTRO="$ros2_distro"
  fi
fi

# setup the required path variables
export ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
export ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
export ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# Check if gnome-terminal exists
if [ -x "$(command -v gnome-terminal)" ]; then
  SHELL_TERM="gnome-terminal --tab -- /bin/bash -c"
  echo $SHELL_TERM
# Check if xterm exists
elif [ -x "$(command -v xterm)" ]; then
  SHELL_TERM="xterm -e"
  echo $SHELL_TERM
fi

printf "\n************* Building ROS2 workspace *************\n\n"

# source the ROS2 environment
if [ -z $ros2_path ]; then
  source /opt/ros/$ROS2_DISTRO/setup.bash
else
  source $ros2_path
fi

# build px4_ros_com package
[ ! -v $verbose ] && colcon_output=$(echo "--event-handlers console_direct+")
cd $ROS2_WS_DIR && colcon build $colcon_output

$SHELL_TERM \
  '''
    # source the ROS2 workspace
    unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

    # source the ROS2 workspace environment so to have it ready to use
    unset ROS_DISTRO && source $ROS2_WS_DIR/install/setup.bash

    exec /bin/bash
  ''' &

# source the ROS2 workspace environment so to have it ready to use
source $ROS2_WS_DIR/install/setup.bash

printf "\nROS2 workspace ready...\n\n"

$SHELL_TERM \
  '''
  # source the ROS2 workspace environment so to have it ready to use
  source $ROS2_WS_DIR/install/setup.bash

  printf "To start the XRCE-DDS agent, use \"micro-ros-agent udp4 --port 2019\"\n\n"
  exec /bin/bash
  ''' &
