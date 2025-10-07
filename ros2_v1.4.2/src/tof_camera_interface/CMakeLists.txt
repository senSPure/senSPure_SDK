cmake_minimum_required(VERSION 3.5)
project(tof_camera_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_options(/wd4819 /wd4996)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# add Messages and Services
set(MSG_FILES
  "msg/CameraType.msg"
  "msg/ConnDevice.msg"
  "msg/FrameInfo.msg"
  "msg/FrameImage.msg"
  "msg/FrameData.msg"
  "msg/ExtTriggerType.msg"
  "msg/ImgOutKind.msg"
  "msg/IntSuppInfo.msg"
  "msg/Version.msg"
  "msg/DeviceInfo.msg"
  "msg/CamFov.msg"
  "msg/Range.msg"
  "msg/Point2d.msg"
  "msg/ModeInfo.msg"
  "msg/PostFiltInfo.msg"
  "msg/LensInfo.msg"
  "msg/MinMaxValue8.msg"
  "msg/MinMaxValue16.msg"
  "msg/MinMaxValue32.msg"
  "msg/ImageFormat.msg"
  "msg/ImageFormats.msg"
  "msg/IntSuppModeType.msg"
  "msg/PlayTime.msg"
  "msg/EventChgProp.msg"
  "msg/EventChgFmt.msg"
  "msg/PostFiltPrm.msg"
  "msg/PosOrgRotation.msg"
  "msg/Notify.msg"
)
# Services
set(SRV_FILES
  "srv/GetDevList.srv"
  "srv/OpenDev.srv"
  "srv/CloseDev.srv"
  "srv/TofCtrl.srv"
  "srv/SetExtTriggerType.srv"
  "srv/SetExtTriggerOffset.srv"
  "srv/SetMode.srv"
  "srv/SetImgKinds.srv"
  "srv/SetLightTimes.srv"
  "srv/SetAEState.srv"
  "srv/SetAEInterval.srv"
  "srv/SetRawSatThreshold.srv"
  "srv/SetIrDarkThreshold.srv"
  "srv/SetIntSupp.srv"
  "srv/SetPlayTarget.srv"
  "srv/SetPlayCtrl.srv"
  "srv/GetDevInfo.srv"
  "srv/GetFov.srv"
  "srv/GetExtTriggerType.srv"
  "srv/GetExtTriggerOffset.srv"
  "srv/GetModeList.srv"
  "srv/GetMode.srv"
  "srv/GetImgKinds.srv"
  "srv/GetImgFormat.srv"
  "srv/GetPostFiltInfo.srv"
  "srv/GetLensInfo.srv"
  "srv/GetLightTimes.srv"
  "srv/GetAEState.srv"
  "srv/GetAEInterval.srv"
  "srv/GetRawSatThreshold.srv"
  "srv/GetIrDarkThreshold.srv"
  "srv/GetIntSuppInfo.srv"
  "srv/GetPlayTarget.srv"
  "srv/GetPlayTime.srv"
  "srv/GetPlayStatus.srv"
  "srv/PsblPostFilt.srv"
  "srv/SetPostFilt.srv"
  "srv/SetPostFiltPrm.srv"
  "srv/PsblLensConv.srv"
  "srv/SetLensConv.srv"
  "srv/SetPcdPos.srv"
  "srv/SetPcdColor.srv"
  "srv/RecordCtrl.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

