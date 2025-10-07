#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/psbl_lens_conv tof_camera_interface/srv/PsblLensConv "{conv_type: $1}"
