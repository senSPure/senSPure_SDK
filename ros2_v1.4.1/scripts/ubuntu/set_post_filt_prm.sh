#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_post_filt_prm tof_camera_interface/srv/SetPostFiltPrm "{param: {median_ksize: $1, bil_ksize: $2, bil_sigma_depth: $3, bil_sigma_ir: $4, bil_sigma_space: $5, flyp_ksize: $6, flyp_log: $7, flyp_thr: $8, flyp_fast_proc: $9}}"
