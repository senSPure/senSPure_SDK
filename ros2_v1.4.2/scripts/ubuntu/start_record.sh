#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/record_ctrl tof_camera_interface/srv/RecordCtrl "{cmd: 0, directory: $1, save_frames: $2, packing_frames: $3, is_crct_dist: $4, is_filt_med: $5, is_filt_bil: $6, is_filt_fly_p: $7}"