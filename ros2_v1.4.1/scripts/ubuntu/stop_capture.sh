#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/tof_ctrl tof_camera_interface/srv/TofCtrl "{cmd: 1}"
