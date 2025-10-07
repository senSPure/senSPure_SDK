#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_ir_dark_threshold tof_camera_interface/srv/GetIrDarkThreshold "{reserved: 0}"
