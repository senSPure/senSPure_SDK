#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/get_img_kinds tof_camera_interface/srv/GetImgKinds "{reserved: 0}"
