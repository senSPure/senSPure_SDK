set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# local variables
set(HEADERS)
set(SOURCES)
set(HEADER_DIR)
set(LINK_LIB)
set(LINK_DIR)

# check build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT ((${CMAKE_BUILD_TYPE} STREQUAL "Debug") OR (${CMAKE_BUILD_TYPE} STREQUAL "Release")))
	message(FATAL_ERROR "CMAKE_BUILD_TYPE=&{CMAKE_BUILD_TYPE} is invalid")
endif()

# set output directory
include(${ENV_DIR}/env_output.cmake)

# compiler options for each platform
if(UNIX)
	message(STATUS "Build environment is Unix")
	include(${ENV_DIR}/env_unix.cmake)
elseif(MSVC)
	message(STATUS "Build environment is Visual C++")
	include(${ENV_DIR}/env_win.cmake)
else()
	message(FATAL_ERROR "Unknown environment")
endif()

# add macros
include(${ENV_DIR}/macro.cmake)
