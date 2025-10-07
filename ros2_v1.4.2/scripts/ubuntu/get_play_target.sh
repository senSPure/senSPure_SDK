#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_play_target tof_camera_interface/srv/GetPlayTarget "{reserved: 0}"
