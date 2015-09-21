#omxcv cmake configuration file

cmake_minimum_required (VERSION 2.8)
project (omxcv)

include (FindPkgConfig)

#Required libraries
find_package (OpenCV 2.4 REQUIRED)
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
CHECK_LIBRARY_EXISTS(tcmalloc malloc "" HAVE_LIBTCMALLOC)
CHECK_LIBRARY_EXISTS(profiler malloc "" HAVE_LIBPROFILER)

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
)

add_library (omxcv ${HEADERS} ${SOURCE})
add_executable (omxcv-test omxcv-test.cpp)

target_link_libraries (omxcv LINK_PRIVATE EGL GLESv2)
target_link_libraries (omxcv LINK_PRIVATE ${LIBAVFORMAT_LIBRARIES} ${LIBAVCODEC_LIBRARIES} ${LIBAVUTIL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries (omxcv LINK_PRIVATE bcm_host openmaxil bcm_host vcos vchiq_arm ilclient)
target_link_libraries (omxcv LINK_PRIVATE ${CMAKE_THREAD_LIBS_INIT})

if (HAVE_LIBTCMALLOC)
	target_link_libraries (omxcv LINK_PUBLIC tcmalloc)
endif (HAVE_LIBTCMALLOC)

if (HAVE_LIBPROFILER)
	target_link_libraries (omxcv LINK_PUBLIC profiler)
endif (HAVE_LIBPROFILER)

#target_link_libraries (omxcv LINK_PRIVATE ${LIBTCMALLOC_LIBRARIES} -lprofiler)

target_link_libraries (omxcv-test LINK_PUBLIC omxcv)
