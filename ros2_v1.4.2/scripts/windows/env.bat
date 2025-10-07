@echo off
if exist "%ROS2_DIRECTORY%/local_setup.bat" (
    call %ROS2_DIRECTORY%/local_setup.bat
) else (
    call c:\dev\ros2-windows\local_setup.bat
)
echo *** ROS_DISTRO: %ROS_DISTRO% ***
call install\local_setup.bat
set ROS_DOMAIN_ID=10
