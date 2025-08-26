cmake_minimum_required(VERSION 3.5)
project(tof_camera_node)

set(COMPONENTS_NAME ${PROJECT_NAME}_components)
set(TOF_CTRL_NAME tof_ctrl_node)
set(POST_FILTER_NAME post_filter_node)
set(LENS_CONV_NAME lens_conv_node)
set(RECORD_NAME record_node)
set(STD_MSG_NAME std_msg_node)

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
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS True)
	add_compile_options(/wd4819 /wd4996)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tof_camera_interface REQUIRED)

# add components source
set(COMP_SRC_FILES
	src/tof_ctrl_component.cpp
	src/post_filter_component.cpp
	src/lens_conv_component.cpp
	src/std_msg_component.cpp
	src/record_component.cpp
)

# add Include path
set(INC_PATH
	include/${PROJECT_NAME}
	lib/include/common
	lib/include/camera
	lib/include/lens
	lib/include/record
	lib/include/postfilter
)

# set sdk library
find_library(SDK_LIB
	NAMES TpTofSdk
	PATHS lib/bin
)
find_library(PSF_LIB
	NAMES PostFilter
	PATHS lib/bin
)

set(DEPENDENCIES
	builtin_interfaces
	std_msgs
	sensor_msgs
	tof_camera_interface
	rclcpp
	rclcpp_components
)

include_directories(${INC_PATH})

# Add components library
add_library(${COMPONENTS_NAME} SHARED ${COMP_SRC_FILES})

rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_node::TofCtrlNode" EXECUTABLE "TofCtrl")
rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_node::PostFilterNode" EXECUTABLE "PostFilter")
rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_node::LensConvNode" EXECUTABLE "LensConv")
rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_node::RecordNode" EXECUTABLE "Record")
rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_node::StdMsgNode" EXECUTABLE "StdMsg")

target_link_libraries(${COMPONENTS_NAME} ${SDK_LIB} ${PSF_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies( ${COMPONENTS_NAME} ${DEPENDENCIES})

# Add node executables
add_executable(            ${TOF_CTRL_NAME} src/tof_ctrl_node.cpp)
target_link_libraries(     ${TOF_CTRL_NAME} ${COMPONENTS_NAME} ${SDK_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies( ${TOF_CTRL_NAME} ${DEPENDENCIES})

add_executable(            ${POST_FILTER_NAME} src/post_filter_node.cpp)
target_link_libraries(     ${POST_FILTER_NAME} ${COMPONENTS_NAME} ${SDK_LIB} ${PSF_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies( ${POST_FILTER_NAME} ${DEPENDENCIES})

add_executable(            ${LENS_CONV_NAME} src/lens_conv_node.cpp)
target_link_libraries(     ${LENS_CONV_NAME} ${COMPONENTS_NAME} ${SDK_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies( ${LENS_CONV_NAME} ${DEPENDENCIES})

add_executable(            ${RECORD_NAME} src/record_node.cpp)
target_link_libraries(     ${RECORD_NAME} ${COMPONENTS_NAME} ${SDK_LIB} ${YAML_CPP_LIBRARIES})
ament_target_dependencies( ${RECORD_NAME} ${DEPENDENCIES})

add_executable(            ${STD_MSG_NAME} src/std_msg_node.cpp)
target_link_libraries(     ${STD_MSG_NAME} ${COMPONENTS_NAME})
ament_target_dependencies( ${STD_MSG_NAME} ${DEPENDENCIES})

# install libraries
install(TARGETS
	${COMPONENTS_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# install executables
set(INS_EXE
	${TOF_CTRL_NAME}
	${POST_FILTER_NAME}
	${LENS_CONV_NAME}
	${RECORD_NAME}
	${STD_MSG_NAME}
)

install(TARGETS
	${INS_EXE}
	DESTINATION lib/${PROJECT_NAME}
)

# install SDK library
file(GLOB SDK_LIBS lib/bin/${CMAKE_SHARED_LIBRARY_PREFIX}*${CMAKE_SHARED_LIBRARY_SUFFIX})
if(MSVC)
	install(FILES
		${SDK_LIBS}
		DESTINATION bin
	)
else()
	install(FILES
		${SDK_LIBS}
		DESTINATION lib
	)
endif()

ament_package()
