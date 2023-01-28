#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: setup_system.bash [option...] \t This script setups the system with the required dependencies." >&2
  echo
  echo -e "\t--ros2_distro \t Set ROS2 distro name (dashing|eloquent|foxy|galactic|rolling). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--clean \t If set, issues 'apt-get autoremove', 'apt-get autoclean' and deletes temp files."
  echo
  exit 0
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

# One can pass the ROS_DISTRO's using the '--ros2_distro' args
unset ROS_DISTRO
if [ -z $ros2_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename.
  # the following distros are the recommended as they are LTS
  case "$(lsb_release -s -c)" in
  "bionic")
    ROS2_DISTRO="dashing"
    ;;
  "focal")
    ROS2_DISTRO="foxy"
    ;;
  *)
    echo "Unsupported version of Ubuntu detected."
    exit 1
    ;;
  esac
else
  ROS2_DISTRO="$ros2_distro"
fi

# Install ROS2 dependencies
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Updating package lists ..."
sudo apt-get -qq update
sudo apt-get -qq dist-upgrade
echo "Installing ROS2 $ROS2_DISTRO and some dependencies..."

sudo apt-get install -y \
  dirmngr \
  gnupg2 \
  python3-colcon-common-extensions \
  python3-dev \
  ros-$ROS2_DISTRO-desktop \
  ros-$ROS2_DISTRO-eigen3-cmake-module \
  ros-$ROS2_DISTRO-launch-testing-ament-cmake \
  ros-$ROS2_DISTRO-rosidl-generator-dds-idl

# Install Python3 packages needed for testing
curl https://bootstrap.pypa.io/get-pip.py | python3 &&
  python3 -m pip install --upgrade pip \
    setuptools \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest \
    pytest-cov \
    pytest-repeat \
    pytest-runner \
    pytest-rerunfailures

# Install Python3 packages for uORB topic generation
python3 -c "import em" || python3 -m pip install --user empy
python3 -c "import genmsg.template_tools" || python3 -m pip install --user pyros-genmsg
python3 -c "import packaging" || python3 -m pip install --user packaging

# Clean residuals
if [ -o clean ]; then
  sudo apt-get -y autoremove &&
    sudo apt-get clean autoclean &&
    sudo rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
fi
