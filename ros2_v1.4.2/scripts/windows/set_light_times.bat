@echo off
ros2 service call /krm/set_light_times tof_camera_interface/srv/SetLightTimes "{count: %1}"
