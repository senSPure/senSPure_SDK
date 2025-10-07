@echo off
ros2 service call /krm/get_post_filt_info tof_camera_interface/srv/GetPostFiltInfo "{reserved: 0}"
