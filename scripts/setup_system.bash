#!/bin/bash
set -e

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: setup_system.bash [option...] \t This script setups the system with the required dependencies." >&2
  echo
  echo -e "\t--ros1_distro \t Set ROS1 distro name (kinetic|melodic). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--ros2_distro \t Set ROS2 distro name (ardent|bouncy|crystal|dashing). If not set, the script will set the ROS_DISTRO env variable based on the Ubuntu codename"
  echo -e "\t--clean \t If set, issues 'apt-get autoremove', 'apt-get autoclean' and deletes temp files."
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
if [ -z $ros1_distro ] && [ -z $ros2_distro ]; then
  # set the ROS_DISTRO variables automatically based on the Ubuntu codename
  case "$(lsb_release -s -c)" in
  "xenial")
    ROS1_DISTRO="kinetic"
    ROS2_DISTRO="ardent"
    ;;
  "bionic")
    ROS1_DISTRO="melodic"
    ROS2_DISTRO="dashing"
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

if [ $ROS2_DISTRO == "ardent" ]; then
  # Install Fast-RTPS (and Fast-RTPS-Gen) 1.6.0
  # Note: Fast-RTPS-Gen was included in the Fast-RTPS release in this version
  wget -q "http://www.eprosima.com/index.php/component/ars/repository/eprosima-fast-rtps/eprosima-fast-rtps-1-6-0/eprosima_fastrtps-1-6-0-linux-tar-gz?format=raw" -O /tmp/eprosima_fastrtps.tar.gz \
    && cd /tmp && tar zxf eprosima_fastrtps.tar.gz \
    && cd eProsima_FastRTPS-1.6.0-Linux \
    && ./configure CXXFLAGS="-g -D__DEBUG" --libdir=/usr/lib \
    && sudo make install \
    && cd $PWD
else
  # Install Gradle 5.6.2 (Required to build Fast-RTPS-Gen)
  wget -q "https://services.gradle.org/distributions/gradle-5.6.2-bin.zip" -O /tmp/gradle-5.6.2-bin.zip \
    && mkdir /opt/gradle \
    && cd /tmp \
    && unzip -d /opt/gradle gradle-5.6.2-bin.zip \
    && echo "export PATH=$PATH:/opt/gradle/gradle-5.6.2/bin" >> ~/.bashrc \
    && source ~/.bashrc \
    && cd $PWD

  # Install Fast-RTPS 1.8.2
  git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x /tmp/FastRTPS-1.8.2 \
    && cd /tmp/FastRTPS-1.8.2 \
    && mkdir build && cd build \
    && cmake -DTHIRDPARTY=ON -DSECURITY=ON .. \
    && sudo make install \
    && cd $PWD

  # Install Fast-RTPS-Gen 1.0.1
  git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.1 /tmp/Fast-RTPS-Gen \
    && cd /tmp/Fast-RTPS-Gen \
    && gradle assemble \
    && sudo cp share/fastrtps/fastrtpsgen.jar /usr/local/share/fastrtps/ \
    && sudo cp scripts/fastrtpsgen /usr/local/bin/ \
    && cd $PWD
fi

# Install ROS1 dependencies
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list"
sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-shadow.list"
wget -qO - http://packages.ros.org/ros.key | sudo apt-key add -

echo "Updating package lists ..."
sudo apt-get -qq update
sudo apt-get -qq dist-upgrade
echo "Installing ROS $ROS1_DISTRO and some dependencies..."

sudo apt-get -y install \
  libeigen3-dev \
  libopencv-dev \
  protobuf-compiler \
  python-catkin-tools \
  python-tk \
  ros-$ROS1_DISTRO-desktop-full \
  ros-$ROS1_DISTRO-gazebo-ros-pkgs \
  ros-$ROS1_DISTRO-rostest \
  ros-$ROS1_DISTRO-rosunit

# Prepare rosdep to install dependencies.
echo "Updating rosdep ..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
# Update rosdep list
rosdep update

# Install Python2 dependencies
pip install -U pip
pip install -I --upgrade future pkgconfig setuptools \
  cerberus empy matplotlib numpy pycollada \
  pyulog pyyaml toml

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO

# Install ROS2 dependencies
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "Updating package lists ..."
sudo sh -c "echo deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -sc) main > /etc/apt/sources.list.d/ros2-latest.list"
sudo apt-get -qq update
sudo apt-get -qq dist-upgrade
echo "Installing ROS2 $ROS2_DISTRO and some dependencies..."

# Install python3-genmsg or download and install from deb source (currently only available in Ubuntu 18.10 and above)
sudo apt-get install -y --quiet python3-genmsg \
  || sudo wget http://mirrors.kernel.org/ubuntu/pool/universe/r/ros-genmsg/python3-genmsg_0.5.11-2_all.deb -P /tmp/ \
    && sudo dpkg -i /tmp/python3-genmsg_0.5.11-2_all.deb \
    && sudo apt-get -y autoremove \
    && sudo apt-get clean autoclean \
    && sudo rm /tmp/python3-genmsg_0.5.11-2_all.deb

sudo apt-get install -y \
  dirmngr \
  gnupg2 \
  python3-colcon-common-extensions \
  python3-dev \
  ros-$ROS2_DISTRO-desktop \
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

# Clean residuals
if [ -o clean ]; then
  sudo apt-get -y autoremove &&
    sudo apt-get clean autoclean &&
    sudo rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*
fi
