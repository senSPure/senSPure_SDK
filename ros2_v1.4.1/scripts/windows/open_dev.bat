@echo off
ros2 service call /krm/open_dev tof_camera_interface/srv/OpenDev "{type: {type: %1}, dev_id: %2}"
