cmake_minimum_required(VERSION 3.5)
project(tof_camera_example)

set(COMPONENTS_NAME ${PROJECT_NAME}_components)
set(SAMPLE_VIEWER_NAME sample_viewer_node)

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

# add sub directories
if(MSVC)
	set(GLFW_BUILD_DOCS OFF)
	set(GLFW_BUILD_EXAMPLES OFF)
	set(GLFW_BUILD_TESTS OFF)
	set(GLFW_USE_HYBRID_HPG OFF)
	set(GLFW_VULKAN_STATIC OFF)
	set(GLFW_INSTALL OFF)
	set(BUILD_SHARED_LIBS ON)
	set(USE_MSVC_RUNTIME_LIBRARY_DLL ON)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/glfw)
endif()

# find third package
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if(MSVC)
	set(GLFW3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/glfw/include)
	set(LINK_LIBS ${LINK_LIBS} OpenGL::GL)
	set(LINK_LIBS ${LINK_LIBS} glfw)
else()
	find_package(glfw3 REQUIRED)
	set(LINK_LIBS ${LINK_LIBS} GL)
	set(LINK_LIBS ${LINK_LIBS} GLU)
	set(LINK_LIBS ${LINK_LIBS} ${CMAKE_DL_LIBS})
	set(LINK_LIBS ${LINK_LIBS} glfw)
endif()

if(MSVC)
	set(OpenCV_DIR C:/tools/opencv/build)
endif()
find_package(OpenCV REQUIRED)
set(HEADER_DIR ${HEADER_DIR} ${OpenCV_INCLUDE_DIRS})
set(LINK_LIBS ${LINK_LIBS} ${OpenCV_LIBS})

find_package(Boost REQUIRED)
set(HEADER_DIR ${HEADER_DIR} ${Boost_INCLUDE_DIRS})
set(LINK_LIBS ${LINK_LIBS} ${Boost_LIBRARIES})

# change imGui configure header
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/imgui_config/imconfig.h
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imconfig.h
	COPYONLY
)

# add components source
set(COMP_SRC_FILES
	src/sample_viewer_component.cpp
	src/ViewerConfig.cpp
	src/ColorTable.cpp
	src/GrayTable.cpp
	src/DrawImage.cpp
	src/DrawDepth.cpp
	src/DrawGray.cpp
	src/DrawRaw.cpp
	src/DisplayRaw.cpp
	src/DrawPcd.cpp
	3rd_pty/imgui/imgui.cpp
	3rd_pty/imgui/imgui_draw.cpp
	3rd_pty/imgui/imgui_tables.cpp
	3rd_pty/imgui/imgui_widgets.cpp
	3rd_pty/imgui/backends/imgui_impl_glfw.cpp
	3rd_pty/imgui/backends/imgui_impl_opengl3.cpp
)

# add Include path
set(INC_PATH
	include/${PROJECT_NAME}
	lib/include/common
	lib/include/camera
	${HEADER_DIR}
	${GLFW3_INCLUDE_DIR}
	3rd_pty/imgui
	3rd_pty/imgui/backends
	3rd_pty/imgui-filebrowser
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

rclcpp_components_register_node(${COMPONENTS_NAME} PLUGIN "tof_camera_example::SampleViewerNode" EXECUTABLE "SampleViewerNode")

#target_link_libraries(${COMPONENTS_NAME} ${SDK_LIB} ${PSF_LIB} ${LINK_LIBS})
target_link_libraries(${COMPONENTS_NAME} ${LINK_LIBS})
ament_target_dependencies( ${COMPONENTS_NAME} ${DEPENDENCIES})

# Add node executables
add_executable(            ${SAMPLE_VIEWER_NAME} src/sample_viewer_node.cpp)
target_link_libraries(     ${SAMPLE_VIEWER_NAME} ${COMPONENTS_NAME} ${LINK_LIBS})
ament_target_dependencies( ${SAMPLE_VIEWER_NAME} ${DEPENDENCIES})

if(UNIX)
	target_compile_definitions(${SAMPLE_VIEWER_NAME} PRIVATE LNX_FUNC)
	target_compile_definitions(${COMPONENTS_NAME} PRIVATE LNX_FUNC)
elseif(WIN32)
	target_compile_definitions(${SAMPLE_VIEWER_NAME} PRIVATE WIN_FUNC)
	target_compile_definitions(${COMPONENTS_NAME} PRIVATE WIN_FUNC)
endif()

# install libraries
install(TARGETS
	${COMPONENTS_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# install executables
set(INS_EXE
	${SAMPLE_VIEWER_NAME}
)

install(TARGETS
	${INS_EXE}
	DESTINATION lib/${PROJECT_NAME}
)

# install configuration file
install(DIRECTORY
	param
	DESTINATION share/${PROJECT_NAME}/
)

# install font file
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/ipag00303/ipag.ttf)
	install(FILES
		3rd_pty/ipag00303/ipag.ttf
		DESTINATION share/${PROJECT_NAME}/3rd_pty
	)
	install(FILES
		3rd_pty/ipag00303/IPA_Font_License_Agreement_v1.0.txt
		DESTINATION share/${PROJECT_NAME}/3rd_pty
	)
else()
	message(STATUS "Japanese font is not exist")
endif()

ament_package()
