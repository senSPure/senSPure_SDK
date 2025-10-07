#!/bin/sh

. ./scripts/ubuntu/env.sh
ros2 service call /krm/set_ext_trigger_type tof_camera_interface/srv/SetExtTriggerType "{type: {ext_trigger_type: $1}}"
