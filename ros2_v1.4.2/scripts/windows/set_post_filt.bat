@echo off
ros2 service call /krm/set_post_filt tof_camera_interface/srv/SetPostFilt "{filt_type: %1, enable: %2}"
