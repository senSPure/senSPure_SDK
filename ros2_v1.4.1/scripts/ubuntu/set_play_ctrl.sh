#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_play_ctrl tof_camera_interface/srv/SetPlayCtrl "{cmd: $1, time: $2}"
