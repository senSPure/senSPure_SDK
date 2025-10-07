@echo off
ros2 service call /krm/get_mode tof_camera_interface/srv/GetMode "{reserved: 0}"
