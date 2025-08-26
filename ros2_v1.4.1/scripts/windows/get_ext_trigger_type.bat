@echo off
ros2 service call /krm/get_ext_trigger_type tof_camera_interface/srv/GetExtTriggerType "{reserved: 0}"
