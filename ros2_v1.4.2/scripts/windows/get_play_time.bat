@echo off
ros2 service call /krm/get_play_time tof_camera_interface/srv/GetPlayTime "{reserved: 0}"
