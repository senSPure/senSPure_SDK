@echo off
ros2 service call /krm/psbl_post_filt tof_camera_interface/srv/PsblPostFilt "{filt_type: %1}"
