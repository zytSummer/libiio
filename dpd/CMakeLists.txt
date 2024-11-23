PROJECT(dpd)
cmake_minimum_required(VERSION 2.8)

option(BUILD_SHARED_LIBS "Build the shared library" OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DADRV9008_2)
set(CMAKE_C_FLAGS "-std=gnu99 -fPIC")

file(GLOB_RECURSE DPD_SRC_FILE "*.c")
file(GLOB_RECURSE DPD_HDR_FILE "*.h")

add_library(dpd STATIC ${DPD_SRC_FILE} ${DPD_HDR_FILE})

#install(FILES src/getopt.h DESTINATION include)

install(TARGETS dpd
    RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)

