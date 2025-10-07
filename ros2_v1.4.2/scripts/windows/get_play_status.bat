@echo off
ros2 service call /krm/get_play_status tof_camera_interface/srv/GetPlayStatus "{reserved: 0}"
