#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_dev_list tof_camera_interface/srv/GetDevList "{type: {type: $1}}"
