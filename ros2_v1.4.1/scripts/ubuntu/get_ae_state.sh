#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_ae_state tof_camera_interface/srv/GetAEState "{reserved: 0}"
