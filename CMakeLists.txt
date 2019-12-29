CMAKE_MINIMUM_REQUIRED(VERSION 2.8) # is enough??

############################################################################################################
# Set /MT compiler flags
set(CompilerFlags
        CMAKE_CXX_FLAGS
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_RELEASE
		CMAKE_CXX_FLAGS_RELWITHDEBINFO
		CMAKE_CXX_FLAGS_MINSIZEREL
        CMAKE_C_FLAGS
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
		CMAKE_C_FLAGS_RELWITHDEBINFO
		CMAKE_C_FLAGS_MINSIZEREL
        )
foreach(CompilerFlag ${CompilerFlags})
  string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
endforeach()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


#####################################################################################
### DOWNLOADING THIRD PARTIES
#####################################################################################
message( STATUS "Downloading 3rd parties:")
set (CMAKE_3RDPARTY_DIR "${CMAKE_BINARY_DIR}/Third parties")
file(MAKE_DIRECTORY ${CMAKE_3RDPARTY_DIR})
# parallel clone?
# ZMQLIB CLONE
set (CMAKE_3RDPARTY_ZMQ_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-zmqlib")
set (CMAKE_3RDPARTY_ZMQ_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-zmqlib-build")
set (ZMQ_TAG "v4.3.2")
message( STATUS " Downloading zmqlib ${ZMQ_TAG} (to ${CMAKE_3RDPARTY_ZMQ_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/zeromq/libzmq.git -b${ZMQ_TAG} \"${CMAKE_3RDPARTY_ZMQ_SRC_DIR}\"" )
message ( STATUS " succeeded.")
# EIGEN CLONE
set (CMAKE_3RDPARTY_EIGEN_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-Eigen3")
#set (CMAKE_3RDPARTY_EIGEN_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-Eigen3-build")
set (Eigen_TAG "3.3.7")
message( STATUS " Downloading eigen3 ${Eigen_TAG} (to ${CMAKE_3RDPARTY_EIGEN_SRC_DIR})...")
EXEC_PROGRAM( "hg clone --verbose https://bitbucket.org/eigen/eigen -r${Eigen_TAG} \"${CMAKE_3RDPARTY_EIGEN_SRC_DIR}\"" )
message ( STATUS " succeeded.")
# CPPZMQ CLONE
set (CMAKE_3RDPARTY_CPPZMQ_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-cppzmq")
#set (CMAKE_3RDPARTY_CPPZMQ_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-cppzmq-build")
set (CPPZMQ_TAG "v4.5.0")
message( STATUS " Downloading cppzmq ${CPPZMQ_TAG} (to ${CMAKE_3RDPARTY_CPPZMQ_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/zeromq/cppzmq.git -b${CPPZMQ_TAG} \"${CMAKE_3RDPARTY_CPPZMQ_SRC_DIR}\"" )
message ( STATUS " succeeded.")
# FLATBUFFERS CLONE
set (CMAKE_3RDPARTY_FLATBUFFERS_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-flatbuffers")
set (CMAKE_3RDPARTY_FLATBUFFERS_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-flatbuffers-build")
set (FLATBUFFERS_TAG "v1.11.0")
message( STATUS " Downloading flatbuffers ${FLATBUFFERS_TAG} (to ${CMAKE_3RDPARTY_FLATBUFFERS_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/google/flatbuffers.git -b${FLATBUFFERS_TAG} \"${CMAKE_3RDPARTY_FLATBUFFERS_SRC_DIR}\"" )
message ( STATUS " succeeded.")
# OPENCV CLONE
set (CMAKE_3RDPARTY_OPENCV_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-opencv")
set (CMAKE_3RDPARTY_OPENCV_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-opencv-build")
set (OPENCV_TAG "4.1.2")
message( STATUS " Downloading opencv ${OPENCV_TAG} (to ${CMAKE_3RDPARTY_OPENCV_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/opencv/opencv.git -b${OPENCV_TAG} \"${CMAKE_3RDPARTY_OPENCV_SRC_DIR}\"" )
message ( STATUS " succeeded.")


set_property(GLOBAL PROPERTY USE_FOLDERS ON)
#####################################################################################
### ADDING THIRD PARTIES
#####################################################################################

add_subdirectory( ${CMAKE_3RDPARTY_ZMQ_SRC_DIR} ${CMAKE_3RDPARTY_ZMQ_BUILD_DIR} ) #EXCLUDE_FROM_ALL ??
set( BUILD_SHARED FALSE CACHE BOOL "" FORCE )
set( BUILD_STATIC TRUE CACHE BOOL "" FORCE )
set( BUILD_TESTS FALSE CACHE BOOL "" FORCE )
set( ENABLE_CPACK FALSE CACHE BOOL "" FORCE )
set( ENABLE_CURVE FALSE CACHE BOOL "" FORCE )
set( ENABLE_DRAFTS FALSE CACHE BOOL "" FORCE )
set( ENABLE_RADIXTREE FALSE CACHE BOOL "" FORCE )
set( LIBZMQ_PEDANTIC FALSE CACHE  BOOL "" FORCE )
set( WITH_PERF_TOOL FALSE CACHE  BOOL "" FORCE )
set( ZMQ_BUILD_TESTS FALSE CACHE BOOL "" FORCE )
set_target_properties(clang-format PROPERTIES FOLDER "3rd Parties/zmq")
set_target_properties(clang-format-check PROPERTIES FOLDER "3rd Parties/zmq")
set_target_properties(clang-format-diff PROPERTIES FOLDER "3rd Parties/zmq")
set_target_properties(libzmq-static PROPERTIES FOLDER "3rd Parties/zmq")

#add_subdirectory( ${CMAKE_3RDPARTY_EIGEN_SRC_DIR} ${CMAKE_3RDPARTY_EIGEN_BUILD_DIR} )
#set( BUILD_TESTING FALSE CACHE BOOL "" FORCE ) #include_install_dir to be set???

#TODO: create manually...

#♥add_subdirectory( ${CMAKE_3RDPARTY_CPPZMQ_SRC_DIR} ${CMAKE_3RDPARTY_CPPZMQ_BUILD_DIR} )
#set( BUILD_TESTING TRUE CACHE BOOL "" FORCE )
#set( CPPZMQ_BUILD_TESTS TRUE CACHE BOOL "" FORCE )
#set( ENABLE_DRAFTS FALSE CACHE BOOL "" FORCE )
#set( ZeroMQ_DIR ${CMAKE_3RDPARTY_CPPZMQ_BUILD_DIR} CACHE STRING "" FORCE )
#set( ZeroMQ_LIBRARY ${ZMQ_BUILD_DIR}/bin CACHE  STRING "" FORCE )
#set( ZeroMQ_STATIC_LIBRARY ${ZMQ_BUILD_DIR}/lib CACHE  STRING "" FORCE )
#☺TODO: set dependency double linking??
#TODO: create manually...

set( BUILD_FLATBUFFERS FALSE)
if (${BUILD_FLATBUFFERS})
	add_subdirectory( ${CMAKE_3RDPARTY_FLATBUFFERS_SRC_DIR} ${CMAKE_3RDPARTY_FLATBUFFERS_BUILD_DIR} )
	set( FLATBUFFERS_BUILD_FLATC TRUE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_BUILD_FLATHASH TRUE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_BUILD_FLATLIB TRUE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_BUILD_GRPCTEST FALSE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_BUILD_SHAREDLIB FALSE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_BUILD_TESTS FALSE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_INSTALL TRUE CACHE BOOL "" FORCE )
	set( FLATBUFFERS_LIBCXX_WITH_CLANG TRUE CACHE  BOOL "" FORCE )
	set( FLATBUFFERS_PACKAGE_DEBIAN FALSE CACHE  BOOL "" FORCE )
	set( FLATBUFFERS_PACKAGE_REDHAT FALSE CACHE BOOL "" FORCE )
endif()

add_subdirectory( ${CMAKE_3RDPARTY_OPENCV_SRC_DIR} ${CMAKE_3RDPARTY_OPENCV_BUILD_DIR} )
set( BUILD_opencv_apps FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_calib3d FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_dnn FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_features2d FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_flann FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_gapi FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_java_bindings_generator FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_ml FALSE CACHE  BOOL "" FORCE )
set( BUILD_opencv_objdetect FALSE CACHE  BOOL "" FORCE )
set( BUILD_opencv_photo FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_python_bindings_generator FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_python_tests FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_stiching FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_ts FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_video FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_videoio FALSE CACHE BOOL "" FORCE )
set( BUILD_opencv_apps FALSE CACHE BOOL "" FORCE )
set( BUILD_JAVA FALSE CACHE  BOOL "" FORCE )
set( BUILD_OPENEXR FALSE CACHE  BOOL "" FORCE )
set( BUILD_PACKAGE FALSE CACHE BOOL "" FORCE )
set( BUILD_PERF_TESTS FALSE CACHE BOOL "" FORCE )
set( BUILD_PROTOBUF FALSE CACHE  BOOL "" FORCE )
set( BUILD_SHARED_LIBS FALSE CACHE  BOOL "" FORCE )
set( BUILD_TESTS FALSE CACHE BOOL "" FORCE )
set( BUILD_TIFF FALSE CACHE BOOL "" FORCE )
set( CPU_DISPATCH "" CACHE STRING "" FORCE )
set( WITH_ADE FALSE CACHE BOOL "" FORCE )
set( WITH_FFMPEG FALSE CACHE BOOL "" FORCE )
set( WITH_EIGEN FALSE CACHE BOOL "" FORCE )
set_target_properties(uninstall PROPERTIES FOLDER "3rd Parties/opencv")
if(0)
set_target_properties(opencv_core_AVX PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_core_AVX2 PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_core_AVX512_SKX PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_core_SSE4_1 PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_core_SSE4_2 PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_imgproc_AVX PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_imgproc_AVX2 PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_imgproc_AVX512_SKX PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
set_target_properties(opencv_imgproc_SSE4_1 PROPERTIES FOLDER "3rd Parties/opencv/dispatched")
endif()
set_target_properties(opencv_modules PROPERTIES FOLDER "3rd Parties/opencv/extra")
set_target_properties(opencv_core PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(opencv_highgui PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(opencv_imgcodecs PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(opencv_imgproc PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(ade PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(opencv_videoio_plugins PROPERTIES FOLDER "3rd Parties/opencv")
set_target_properties(IlmImf PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(ippiw PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(ittnotify PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(libjasper PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(libjpeg-turbo PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(libpng PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(libtiff PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(libwebp PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(quirc PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
set_target_properties(zlib PROPERTIES FOLDER "3rd Parties/opencv/3rdparty")
#videoioplugins wtf?

macro(requires_eigen NAME)
	include_directories ( ${CMAKE_3RDPARTY_EIGEN_SRC_DIR}	)
	#set ( a  )
	#include_directories ( ${CMAKE_3RDPARTY_DIR}/Ext-Eigen3 )
	#target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

macro(requires_zmq NAME)
	include_directories (
		${CMAKE_3RDPARTY_ZMQ_SRC_DIR}/include
		${CMAKE_3RDPARTY_ZMQ_SRC_DIR}/src
		${CMAKE_3RDPARTY_CPPZMQ_SRC_DIR}
		)
	target_link_libraries (${NAME} libzmq-static)
	add_definitions(-DZMQ_STATIC)
endmacro(requires_cppzmq)

macro(requires_flatbuffers NAME)
	include_directories ( ${CMAKE_3RDPARTY_FLATBUFFERS_SRC_DIR}/include )
endmacro(requires_flatbuffers)

macro(requires_opencv NAME)
	include_directories( #${OpenCV_INCLUDE_DIRS} 
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/core/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/highgui/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/imgproc/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/imgcodecs/include
		${CMAKE_BINARY_DIR}
		)
	target_link_libraries ( ${NAME} opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs)
endmacro(requires_opencv)

PROJECT(SensorFusion)

# TODO: add gtest, linux compatibility?

#####################################################################################
#####################Minden kell ebbõl?##############################################
# use folders in IDEs supporting it
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "")


if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
    #This is required to find sodium package for libzmq linking
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif()

# if(NOT LINUX) should work, too, if you need that
if(LINUX) 
	message(STATUS ">>> Linux")
    set (CMAKE_CXX_STANDARD 11)
else()
    if(WIN32)
        # for Windows operating system in general
        # on *nixes we need this to use C++11 
        if (MSVC_VERSION GREATER_EQUAL "1900")
            include(CheckCXXCompilerFlag)
            CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
            if (_cpp_latest_flag_supported)
                add_compile_options("/std:c++latest")
            endif()
        endif()
    endif()
endif()

# where to put the resulting binaries?
set (APPLICATION Bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${APPLICATION})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${APPLICATION})
#####################################################################################
set (CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo CACHE STRING "Configuration types" FORCE)



## Generate Compiler options
include (cmake/CompilerFlags.cmake)

option(ADD_PLOTTER "To use OpenCV-based plotters" TRUE)

add_subdirectory (Lib_Common)

if (ADD_PLOTTER)
	add_subdirectory (Lib_Plotter)
	add_subdirectory (App_Plotter)
endif()
add_subdirectory (Lib_SensorFusion)
add_subdirectory (Lib_Communication)
add_subdirectory (App_ZMQSub)
add_subdirectory (App_ZMQPub)