#!/bin/sh

# argument
#	launch_alone.py			: TofCtrl + PostFilter + LensConv
#	launch_rviz2.py			: TofCtrl + PostFilter + LensConv + StdMsg + rviz2
#	launch_rviz2_record.py	: TofCtrl + PostFilter + LensConv + StdMsg + Record + rviz2
#	launch_viewer.py		: TofCtrl + PostFilter + LensConv + Record + SampleViewer

. ./scripts/ubuntu/env.sh
ros2 launch tof_camera_param $1
