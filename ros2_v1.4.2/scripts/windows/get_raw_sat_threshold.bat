@echo off
ros2 service call /krm/get_raw_sat_threshold tof_camera_interface/srv/GetRawSatThreshold "{reserved: 0}"
