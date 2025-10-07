@echo off
ros2 service call /krm/set_ae_interval tof_camera_interface/srv/SetAEInterval "{interval: %1}"
