#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_lens_conv tof_camera_interface/srv/SetLensConv "{conv_type: $1, enable: $2}"
