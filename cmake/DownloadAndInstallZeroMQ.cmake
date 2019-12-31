#### ZeroMQ
set (CMAKE_3RDPARTY_ZMQ_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-ZMQ")
set (CMAKE_3RDPARTY_ZMQ_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-ZMQ-build")
# ZMQ CLONE
set (ZMQ_TAG "v4.3.2")
message( STATUS "Downloading ZMQ ${ZMQ_TAG} (to ${CMAKE_3RDPARTY_ZMQ_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/zeromq/libzmq.git -b${ZMQ_TAG} \"${CMAKE_3RDPARTY_ZMQ_SRC_DIR}\"" )
message ( STATUS " succeeded. CMake configure started....")
# Call configure, generate and build (Release & Debug) on the externals
message(STATUS "Generating solution/makefile for ZMQ")
EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_3RDPARTY_ZMQ_SRC_DIR}\" ARGS 
	\"${GENERATOR_ARGUMENT}\" \"-S${CMAKE_3RDPARTY_ZMQ_SRC_DIR}\" \"-B${CMAKE_3RDPARTY_ZMQ_BUILD_DIR}\"
	-DBUILD_SHARED=TRUE
	-DBUILD_STATIC=TRUE
	-DBUILD_TESTS=FALSE
	-DENABLE_CPACK=FALSE
	-DENABLE_CURVE=FALSE
	-DENABLE_DRAFTS=FALSE
	-DENABLE_RADIX_TREE=FALSE
	-DLIBZMQ_PEDANTIC=FALSE
	-DWITH_PERF_TOOL=FALSE
	-DZMQ_BUILD_TESTS=FALSE
	-DCMAKE_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\"
	${EXTERNAL_FLAG_SETTER}
	)
message(STATUS "Building ZeroMQ (RELEASE). It can take some time...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--build \"${CMAKE_3RDPARTY_ZMQ_BUILD_DIR}\" --config Release )
message(STATUS "Building ZeroMQ (DEBUG). It can take some time...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--build \"${CMAKE_3RDPARTY_ZMQ_BUILD_DIR}\" --config Debug )
message(STATUS "Installing ZeroMQ (DEBUG)...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--install \"${CMAKE_3RDPARTY_ZMQ_BUILD_DIR}\" --config Release )
message(STATUS "Installing ZeroMQ (RELEASE)...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--install \"${CMAKE_3RDPARTY_ZMQ_BUILD_DIR}\" --config Debug )
message(STATUS "done.")