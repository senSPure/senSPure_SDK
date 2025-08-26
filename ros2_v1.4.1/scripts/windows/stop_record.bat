@echo off
ros2 service call /krm/record_ctrl tof_camera_interface/srv/RecordCtrl "{cmd: 1, directory: "", save_frames: 0, packing_frames: 0, is_crct_dist: 0, is_filt_med: 0, is_filt_bil: 0, is_filt_fly_p: 0}"
