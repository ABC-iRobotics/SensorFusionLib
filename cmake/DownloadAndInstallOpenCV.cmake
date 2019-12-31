set (CMAKE_3RDPARTY_OPENCV_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-opencv")
set (CMAKE_3RDPARTY_OPENCV_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-opencv-build")

# OPENCV CLONE
set (OPENCV_TAG "4.1.2")
message( STATUS "Downloading opencv ${OPENCV_TAG} (to ${CMAKE_3RDPARTY_OPENCV_SRC_DIR})...")
EXEC_PROGRAM( "git clone https://github.com/opencv/opencv.git -b${OPENCV_TAG} \"${CMAKE_3RDPARTY_OPENCV_SRC_DIR}\"" )
message ( STATUS " succeeded. CMake configure started....")
# Call configure, generate and build (Release & Debug) on the externals
message(STATUS "Generating makefile/solution for opencv")
EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_3RDPARTY_OPENCV_SRC_DIR}\" ARGS 
	\"${GENERATOR_ARGUMENT}\" \"-S${CMAKE_3RDPARTY_OPENCV_SRC_DIR}\" \"-B${CMAKE_3RDPARTY_OPENCV_BUILD_DIR}\"
	#-DBUILD_SUBTREE=1
	-DBUILD_opencv_apps=FALSE
	-DBUILD_opencv_calib3d=FALSE
	-DBUILD_opencv_dnn=FALSE
	-DBUILD_opencv_features2d=FALSE
	-DBUILD_opencv_flann=FALSE
	-DBUILD_opencv_gapi=FALSE
	-DBUILD_opencv_java_bindings_generator=FALSE
	-DBUILD_opencv_ml=FALSE
	-DBUILD_opencv_objdetect=FALSE
	-DBUILD_opencv_photo=FALSE
	-DBUILD_opencv_python_bindings_generator=FALSE
	-DBUILD_opencv_python_tests=FALSE
	-DBUILD_opencv_stiching=FALSE
	-DBUILD_opencv_ts=FALSE
	-DBUILD_opencv_video=FALSE
	-DBUILD_opencv_videoio=FALSE
	-DBUILD_opencv_apps=FALSE
	-DBUILD_JAVA=FALSE
	-DBUILD_OPENEXR=FALSE
	-DBUILD_PACKAGE=FALSE
	-DBUILD_PERF_TESTS=FALSE
	-DBUILD_PROTOBUF=FALSE
	-DBUILD_SHARED_LIBS=FALSE
	-DBUILD_TESTS=FALSE
	-DBUILD_TIFF=FALSE
	-DWITH_ADE=FALSE
	-DWITH_FFMPEG=FALSE
	-DWITH_EIGEN=FALSE
	-DCPU_DISPATCH=""
	-DCMAKE_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\"
	${EXTERNAL_FLAG_SETTER}
	)
message(STATUS "Building opencv (RELEASE). It can take some time...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--build \"${CMAKE_3RDPARTY_OPENCV_BUILD_DIR}\" --config Release )
message(STATUS "Building opencv (DEBUG). It can take some time...")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--build \"${CMAKE_3RDPARTY_OPENCV_BUILD_DIR}\" --config Debug )
message(STATUS "Installing opencv (RELEASE).")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--install \"${CMAKE_3RDPARTY_OPENCV_BUILD_DIR}\" --config Release )
message(STATUS "Installing opencv (DEBUG).")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--install \"${CMAKE_3RDPARTY_OPENCV_BUILD_DIR}\" --config Debug )
message(STATUS "done.")