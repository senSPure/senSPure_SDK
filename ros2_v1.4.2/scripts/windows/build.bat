@echo off
if exist "%ROS2_DIRECTORY%/local_setup.bat" (
    call %ROS2_DIRECTORY%/local_setup.bat
) else (
    call c:\dev\ros2-windows\local_setup.bat
)
echo *** ROS_DISTRO: %ROS_DISTRO% ***
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat"
colcon build
