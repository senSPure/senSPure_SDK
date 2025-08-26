@echo off
ros2 service call /krm/set_img_kinds tof_camera_interface/srv/SetImgKinds "{img_out: {img_out_kind: %1}}"
