#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_int_supp tof_camera_interface/srv/SetIntSupp "{int_supp_mode: {int_supp_mode_type: $1}, int_supp_prm_m: $2, int_supp_prm_a1: $3, int_supp_prm_a2: $4, int_supp_prm_a3: $5}"
