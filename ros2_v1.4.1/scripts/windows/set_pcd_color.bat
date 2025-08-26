@echo off
ros2 service call /krm/set_pcd_color tof_camera_interface/srv/SetPcdColor "{color: %1}"
