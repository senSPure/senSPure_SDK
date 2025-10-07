#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_play_target tof_camera_interface/srv/SetPlayTarget "{directory: $1}"
