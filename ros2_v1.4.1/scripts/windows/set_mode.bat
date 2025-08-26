@echo off
ros2 service call /krm/set_mode tof_camera_interface/srv/SetMode "{mode: %1}"
