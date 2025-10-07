# install public header file macro
macro(ins_header dir file)
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${file}
		${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${dir}/${file}
		COPYONLY
	)
endmacro()

# install public header file macro
macro(ins_sub_header dir sub file)
	configure_file(
		${CMAKE_CURRENT_SOURCE_DIR}/${dir}/${sub}/${file}
		${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/${dir}/${file}
		COPYONLY
	)
endmacro()

# add header path macro
macro(inc_dir dir)
	set(HEADER_DIR ${HEADER_DIR} ${dir})
endmacro()

# add sources and headers macro
macro(add_src dir)
	file(GLOB SUB_HEADERS ${dir}/*.h)
	file(GLOB SUB_SOURCES ${dir}/*.cpp)
	set(HEADERS ${HEADERS} ${SUB_HEADERS})
	set(SOURCES ${SOURCES} ${SUB_SOURCES})
	unset(SUB_HEADERS)
	unset(SUB_SOURCES)
	inc_dir(${dir})
endmacro()

# add SDK public headers
macro(inc_sdk_header)
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY})
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/common)
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/pipeline)
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/camera)
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/lens)
	set(HEADER_DIR ${HEADER_DIR} ${CMAKE_INCLUDE_OUTPUT_DIRECTORY}/record)
endmacro()
