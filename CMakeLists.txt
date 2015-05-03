#omxcv cmake configuration file

cmake_minimum_required (VERSION 2.8)
project (omxcv)

include (FindPkgConfig)

#Required libraries
find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIR})
find_package (Threads REQUIRED) # For pthreads

pkg_check_modules (LIBAVFORMAT libavformat REQUIRED)
pkg_check_modules (LIBAVCODEC libavcodec REQUIRED)
pkg_check_modules (LIBAVUTIL libavutil REQUIRED)

include_directories (${LIBAVFORMAT_INCLUDE_DIRS})
link_directories (${LIBAVFORMAT_LIBRARY_DIRS})
include_directories (${LIBAVCODEC_INCLUDE_DIRS})
link_directories (${LIBAVCODEC_LIBRARY_DIRS})
include_directories (${LIBAVUTIL_INCLUDE_DIRS})
link_directories (${LIBAVUTIL_LIBRARY_DIRS})

#Optional libraries
pkg_check_modules (LIBTCMALLOC libtcmalloc)
include_directories (${LIBTCMALLOC_INCLUDE_DIRS}})
link_directories (${LIBTCMALLOC_LIBRARY_DIRS})

#Raspberry Pi specific includes.
#Requires the installation of the userland package.
include_directories (/opt/vc/include /opt/vc/src/hello_pi/libs/ilclient /opt/vc/include/interface/vcos/pthreads /opt/vc/include/interface/vmcs_host/linux)
link_directories (/opt/vc/lib /opt/vc/src/hello_pi/libs/ilclient)

#Compile options
if (CMAKE_COMPILER_IS_GNUCXX)
	set (CMAKE_CXX_FLAGS "-Wall -Wno-variadic-macros -std=c++11 -pedantic -gdwarf-3")
endif (CMAKE_COMPILER_IS_GNUCXX)

#Source files
set (HEADERS
    omxcv.h
    image_gpu.h
)

set (SOURCE
    image_gpu.cpp
    omxcv.cpp
    bgr2rgb.s
)

add_executable (omxcv ${HEADERS} ${SOURCE})
set_property(SOURCE bgr2rgb.s PROPERTY LANGUAGE C)

target_link_libraries (omxcv EGL GLESv2)
target_link_libraries (omxcv ${LIBAVFORMAT_LIBRARIES} ${LIBAVCODEC_LIBRARIES} ${LIBAVUTIL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (omxcv bcm_host openmaxil bcm_host vcos vchiq_arm ilclient)
target_link_libraries (omxcv ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries (omxcv ${LIBTCMALLOC_LIBRARIES} -lprofiler)
