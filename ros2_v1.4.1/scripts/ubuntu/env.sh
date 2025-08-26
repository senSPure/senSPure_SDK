#!/bin/sh

if [ -n "$ROS2_DIRECTORY" -a -f "$ROS2_DIRECTORY"/setup.sh ]; then
    echo "*** using: $ROS2_DIRECTORY/setup.sh ***"
    . $ROS2_DIRECTORY/setup.sh
else
    . /opt/ros/humble/setup.sh
fi
echo "*** ROS_DISTRO: $ROS_DISTRO ***"
. install/local_setup.sh
export ROS_DOMAIN_ID=10
