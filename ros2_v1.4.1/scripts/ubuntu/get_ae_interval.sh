#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_ae_interval tof_camera_interface/srv/GetAEInterval "{reserved: 0}"
