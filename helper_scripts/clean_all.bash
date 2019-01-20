#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: clean_all.bash [option...] \t This script cleans the build for both workspaces" >&2
  echo
  echo -e "\t--delete_ros1_bridge \t Deletes the ros1_bridge repository under the ROS2 workspace"
  echo -e "\t--ros1_ws_dir \t\t Location of the ROS(1) workspace where one has cloned px4_ros_com 'ros1' branch. Default: $HOME/px4_ros_com_ros1"
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

SCRIPT_DIR=$PWD

echo "Build cleansing..."
CLEAN_HISTORY=0

# ROS2 dirs
ROS2_REPO_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)
ROS2_PKG_SRC_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")" && pwd)
ROS2_MSGS_REPO_DIR=$(cd "$(dirname "$ROS2_REPO_DIR")/px4_ros_com/px4_msgs" && pwd)
ROS2_WS_DIR=$(cd "$(dirname "$ROS2_PKG_SRC_DIR")" && pwd)

# ROS1 dirs (one can pass the ROS1 workspace dir using '--ros1_ws_dir <ws_dir>')
ROS1_WS_DIR=${ros1_ws_dir:-"$(cd "$HOME/px4_ros_com_ros1" && pwd)"}
ROS1_REPO_DIR=$(cd $ROS1_WS_DIR/src/px4_ros_com && pwd)

# clean generated msgs and RTPS ID map file on the px4_ros_com ROS2 side
cd $ROS2_MSGS_REPO_DIR/msg && sudo find . -name "*.msg" -type f -delete
if [ -f $ROS2_MSGS_REPO_DIR/msg/templates/uorb_rtps_message_ids.yaml ]; then
  cd $ROS2_MSGS_REPO_DIR/msg/templates && sudo rm uorb_rtps_message_ids.yaml
  echo -e "\t - Deleted $ROS2_MSGS_REPO_DIR/msg/templates/uorb_rtps_message_ids.yaml"
  CLEAN_HISTORY=1
fi

# clean micrortps_agent generated source code
if [ -d $ROS2_REPO_DIR/px4_ros_com/src/micrortps_agent ]; then
  cd $ROS2_REPO_DIR/px4_ros_com/src && sudo rm -rf micrortps_agent
  echo -e "\t - Deleted $ROS2_REPO_DIR/px4_ros_com/src/micrortps_agent"
  CLEAN_HISTORY=1
fi

# if '--delete_ros1_bridge' set, delete the ros1_bridge repo
if [ -v $delete_ros1_bridge ] && [ -d $ROS2_PKG_SRC_DIR/ros1_bridge ]; then
  cd $ROS2_PKG_SRC_DIR && sudo rm -rf ros1_bridge
  echo -e "\t - Deleted $ROS2_PKG_SRC_DIR/ros1_bridge"
  CLEAN_HISTORY=1
fi

# delete build, install and log folders in the ROS2 workspace
dirs=("$ROS2_WS_DIR/build" "$ROS2_WS_DIR/install" "$ROS2_WS_DIR/log")
for dir in "${dirs[@]}"; do
  if [ -d $dir ]; then
    sudo rm -rf $dir
    echo -e "\t - Deleted $dir"
    CLEAN_HISTORY=1
  fi
done

# clean generated msgs on the ROS1 side
cd $ROS1_REPO_DIR/msg && sudo find . -name "*.msg" -type f -delete

# delete build, install and log folders in the ROS1 workspace
dirs=("$ROS1_WS_DIR/build" "$ROS1_WS_DIR/install" "$ROS1_WS_DIR/log")
for dir in "${dirs[@]}"; do
  if [ -d $dir ]; then
    sudo rm -rf $dir
    echo -e "\t - Deleted $dir"
    CLEAN_HISTORY=1
  fi
done

if [ $CLEAN_HISTORY = 1 ]; then
  echo "Done cleaning build!..."
else
  echo "Nothing to clean!"
fi

cd $SCRIPT_DIR
