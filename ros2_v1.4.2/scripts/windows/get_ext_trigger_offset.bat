@echo off
ros2 service call /krm/get_ext_trigger_offset tof_camera_interface/srv/GetExtTriggerOffset "{reserved: 0}"
