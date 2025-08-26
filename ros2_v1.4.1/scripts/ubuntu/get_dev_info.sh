#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_dev_info tof_camera_interface/srv/GetDevInfo "{reserved: 0}"
