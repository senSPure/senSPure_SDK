@echo off
ros2 service call /krm/get_int_supp_info tof_camera_interface/srv/GetIntSuppInfo "{reserved: 0}"
