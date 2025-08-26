@echo off
rem argument
rem	launch_alone.py			: TofCtrl + PostFilter + LensConv
rem	launch_rviz2.py			: TofCtrl + PostFilter + LensConv + StdMsg + rviz2
rem	launch_rviz2_record.py	: TofCtrl + PostFilter + LensConv + StdMsg + Record + rviz2
rem	launch_viewer.py		: TofCtrl + PostFilter + LensConv + Record + SampleViewer

if exist "%ROS2_DIRECTORY%/local_setup.bat" (
    call %ROS2_DIRECTORY%/local_setup.bat
) else (
    call c:\dev\ros2-windows\local_setup.bat
)
echo *** ROS_DISTRO: %ROS_DISTRO% ***
call install\local_setup.bat
set ROS_DOMAIN_ID=10
ros2 launch tof_camera_param %1
