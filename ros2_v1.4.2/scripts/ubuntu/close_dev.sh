#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/close_dev tof_camera_interface/srv/CloseDev "{reserved: 0}"
