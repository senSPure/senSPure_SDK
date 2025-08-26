#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_mode_list tof_camera_interface/srv/GetModeList "{reserved: 0}"
