@echo off
ros2 service call /krm/get_img_format tof_camera_interface/srv/GetImgFormat "{reserved: 0}"
