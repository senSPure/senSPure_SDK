#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_ir_dark_threshold tof_camera_interface/srv/SetIrDarkThreshold "{ir_threshold: $1}"
