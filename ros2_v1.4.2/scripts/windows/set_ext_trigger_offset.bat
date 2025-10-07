@echo off
ros2 service call /krm/set_ext_trigger_offset tof_camera_interface/srv/SetExtTriggerOffset "{offset: %1}"
