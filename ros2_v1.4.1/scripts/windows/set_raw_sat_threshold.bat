@echo off
ros2 service call /krm/set_raw_sat_threshold tof_camera_interface/srv/SetRawSatThreshold "{raw_threshold: %1}"
