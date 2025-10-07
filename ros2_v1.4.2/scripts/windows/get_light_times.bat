@echo off
ros2 service call /krm/get_light_times tof_camera_interface/srv/GetLightTimes "{reserved: 0}"
