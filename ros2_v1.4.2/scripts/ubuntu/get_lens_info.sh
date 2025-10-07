#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_lens_info tof_camera_interface/srv/GetLensInfo "{reserved: 0}"
