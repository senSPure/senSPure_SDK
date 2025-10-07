@echo off
ros2 service call /krm/set_pcd_pos tof_camera_interface/srv/SetPcdPos "{pos: {offset_x: %1, offset_y: %2, offset_z: %3, rotation_pitch: %4, rotation_yaw: %5, rotation_roll: %6}}"
