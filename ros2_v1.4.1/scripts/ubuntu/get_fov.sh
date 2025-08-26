#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_fov tof_camera_interface/srv/GetFov "{reserved: 0}"
