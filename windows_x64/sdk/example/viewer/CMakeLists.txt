cmake_minimum_required(VERSION 3.12)
project(TOPPAN_ToF_Viewer LANGUAGES CXX C)

set(ENV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
include(${ENV_DIR}/env.cmake)

# find third package
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if(WIN32)
	set(GLFW3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/glfw/include)
	set(LINK_LIBS ${LINK_LIBS} OpenGL::GL)
	find_library(GLFW_LIB NAMES glfw3dll PATHS ${CMAKE_BINARY_DIR}/bin/Release)
	set(LINK_LIBS ${LINK_LIBS} ${GLFW_LIB})
elseif(UNIX)
	find_package(glfw3 REQUIRED)
	set(LINK_LIBS ${LINK_LIBS} GL)
	set(LINK_LIBS ${LINK_LIBS} GLU)
	set(LINK_LIBS ${LINK_LIBS} ${CMAKE_DL_LIBS})
	set(LINK_LIBS ${LINK_LIBS} glfw)
endif()

if(WIN32)
	set(OpenCV_DIR C:/tools/opencv/build)
endif()
find_package(OpenCV REQUIRED)
set(HEADER_DIR ${HEADER_DIR} ${OpenCV_INCLUDE_DIRS})
set(LINK_LIBS ${LINK_LIBS} ${OpenCV_LIBS})

find_package(Boost REQUIRED)
set(HEADER_DIR ${HEADER_DIR} ${Boost_INCLUDE_DIRS})
set(LINK_LIBS ${LINK_LIBS} ${Boost_LIBRARIES})

# add SDK library
inc_sdk_header()
if(MSVC)
	find_library(SDK_LIB NAMES TpTofSdk PATHS ${CMAKE_BINARY_DIR}/bin/Release)
else()
	find_library(SDK_LIB NAMES TpTofSdk PATHS ${CMAKE_BINARY_DIR}/bin)
endif()
set(LINK_LIBS ${LINK_LIBS} ${SDK_LIB})

# add Post Filter library
set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/postfilter)
if(MSVC)
	find_library(POST_FILT_LIB NAMES PostFilter PATHS ${CMAKE_BINARY_DIR}/bin/Release)
else()
	find_library(POST_FILT_LIB NAMES PostFilter PATHS ${CMAKE_BINARY_DIR}/bin)
endif()
set(LINK_LIBS ${LINK_LIBS} ${POST_FILT_LIB})

# add sources and headers
file(GLOB HEADERS *.h)
file(GLOB SOURCES *.cpp)

# change imGui configure header
configure_file(
	${CMAKE_CURRENT_SOURCE_DIR}/imgui_config/imconfig.h
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imconfig.h
	COPYONLY
)

# add imGui sources
set(SOURCES ${SOURCES}
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imgui.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imgui_draw.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imgui_tables.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/imgui_widgets.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/backends/imgui_impl_glfw.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/backends/imgui_impl_opengl3.cpp
)
set(HEADER_DIR ${HEADER_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}
	${GLFW3_INCLUDE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui/backends
	${CMAKE_CURRENT_SOURCE_DIR}/3rd_pty/imgui-filebrowser
)
include(pipeline/pipeline.cmake)
include(draw/draw.cmake)

# make executer
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} ${VSRC})
target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIR})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

if(UNIX)
	target_compile_definitions(${PROJECT_NAME} PRIVATE LNX_FUNC)
elseif(WIN32)
	target_compile_definitions(${PROJECT_NAME} PRIVATE WIN_FUNC)
endif()


# install config file
if(MSVC)
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/ViewerConfig.json
		${CMAKE_BINARY_DIR}/example/viewer/ViewerConfig.json
		COPYONLY
	)
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/ViewerConfig.json
		${CMAKE_BINARY_DIR}/bin/Release/ViewerConfig.json
		COPYONLY
	)
else()
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/ViewerConfig.json
		${CMAKE_BINARY_DIR}/ViewerConfig.json
		COPYONLY
	)
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/ViewerConfig.json
		${CMAKE_BINARY_DIR}/bin/ViewerConfig.json
		COPYONLY
	)
endif()
