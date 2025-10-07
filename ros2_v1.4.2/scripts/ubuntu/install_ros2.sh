#!/bin/sh

# This is an installation script of ROS2 Humble Halksbill for Ubuntu 20.04
# see: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

set -x

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

sudo python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-import-order \
  flake8-quotes \
  "pytest>=5.3" \
  pytest-repeat \
  pytest-rerunfailures

#
# Preparing ROS2 Humble Sources
#

umask 022
sudo rm -fr /opt/ros/humble /opt/ros/humble-ws
sudo mkdir -p /opt/ros/humble /opt/ros/humble-ws/src
sudo chown -R `id -un`.`id -gn` /opt/ros/humble /opt/ros/humble-ws

cd /opt/ros/humble-ws
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

#
# Update system packages for ROS2 Humble
#

# Strongly recommended:
#sudo apt upgrade

# Install dependencies using rosdep
sudo rosdep init
rosdep update
rosdep install --rosdistro humble --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Ensure followings are installed:
sudo apt install libasio-dev -y
sudo apt install libtinyxml2-dev -y

#
# Now we build ROS2 Humble
#

colcon build --install-base /opt/ros/humble

sudo chown -R root.root /opt/ros/humble

# https://colcon.readthedocs.io/en/released/user/installation.html
sudo apt install python3-colcon-common-extensions
