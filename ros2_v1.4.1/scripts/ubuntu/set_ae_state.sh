#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_ae_state tof_camera_interface/srv/SetAEState "{enable: $1}"
