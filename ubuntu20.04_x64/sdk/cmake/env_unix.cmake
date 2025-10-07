# set gcc options
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -Wextra -pedantic -D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O2")
set(CMAKE_CXX_FLAGS_RELEASE "-s -O2")

# find pthread
find_package(Threads REQUIRED)
set(LINK_LIBS ${LINK_LIBS} ${CMAKE_THREAD_LIBS_INIT})
