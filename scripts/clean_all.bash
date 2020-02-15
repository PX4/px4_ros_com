#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: clean_all.bash [option...] \t This script cleans the build of the workspace(s)" >&2
  echo
  echo -e "\t--delete_ros1_bridge \t Deletes the ros1_bridge repository under the ROS2 workspace"
  echo -e "\t--delete_ros1_ws_build \t\t Deletes 'build', 'devel', 'install' and 'logs' folder on the given location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. If none is set, defaults to $HOME/px4_ros_com_ros1"
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

echo "Build cleansing..."
CLEAN_HISTORY=0

# ROS2 dirs
ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS2_WS_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
ROS2_WS_DIR=$(cd "$(dirname "$ROS2_WS_SRC_DIR")" && pwd)

# ROS (1) workspace dir to delete generated build files (one can pass the workspace dir using '--delete_ros1_ws_build <ws_dir>')
ROS1_WS_DIR=${delete_ros1_ws_build:-""}

echo -e "\t - ROS2:"
# clean micrortps_agent generated source code
if [ -d $ROS2_WS_SRC_DIR/px4_ros_com/src/micrortps_agent ]; then
  cd $ROS2_WS_SRC_DIR/px4_ros_com/src && sudo rm -rf micrortps_agent
  echo -e "\t -- Deleted $ROS2_WS_SRC_DIR/px4_ros_com/src/micrortps_agent"
  CLEAN_HISTORY=1
fi

# if '--delete_ros1_bridge' set, delete the ros1_bridge repo
if [ ! -v $delete_ros1_bridge ] && [ -d $ROS2_WS_SRC_DIR/ros1_bridge ]; then
  cd $ROS2_WS_SRC_DIR && sudo rm -rf ros1_bridge
  echo -e "\t -- Deleted $ROS2_WS_SRC_DIR/ros1_bridge"
  CLEAN_HISTORY=1
fi

# delete build, install and log folders in the ROS2 workspace
dirs=("$ROS2_WS_DIR/build" "$ROS2_WS_DIR/install" "$ROS2_WS_DIR/log")
for dir in "${dirs[@]}"; do
  if [ -d $dir ]; then
    sudo rm -rf $dir
    echo -e "\t -- Deleted $dir"
    CLEAN_HISTORY=1
  fi
done

# delete build, devel (if built with catkin), install and log folders in the ROS (1) workspace
echo -e "\t - ROS(1):"
if [ ! -v $ROS1_WS_DIR ]; then
  dirs=("$ROS1_WS_DIR/build" "$ROS1_WS_DIR/devel" "$ROS1_WS_DIR/install" "$ROS1_WS_DIR/log")
  for dir in "${dirs[@]}"; do
    if [ -d $dir ]; then
      sudo rm -rf $dir
      echo -e "\t -- Deleted $dir"
      CLEAN_HISTORY=1
    fi
  done
fi

if [ $CLEAN_HISTORY = 1 ]; then
  echo "Done cleaning build!..."
else
  echo "Nothing to clean!"
fi

cd $SCRIPT_DIR
