set (CMAKE_3RDPARTY_EIGEN_SRC_DIR "${CMAKE_3RDPARTY_DIR}/Ext-Eigen3")
set (CMAKE_3RDPARTY_EIGEN_BUILD_DIR "${CMAKE_3RDPARTY_DIR}/Ext-Eigen3-build")
# Eigen3 CLONE
set (EIGEN3_TAG "3.3.7")
message( STATUS "Downloading Eigen3 ${EIGEN3_TAG} (to ${CMAKE_3RDPARTY_EIGEN_SRC_DIR})...")
EXEC_PROGRAM( "hg clone --verbose https://bitbucket.org/eigen/eigen -r${EIGEN3_TAG} \"${CMAKE_3RDPARTY_EIGEN_SRC_DIR}\"" )
message ( STATUS " succeeded. CMake configure started....")
# Call configure, generate and build (Release & Debug) on the externals
message(STATUS "Generating makefile/solution for Eigen3")
EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_3RDPARTY_EIGEN_SRC_DIR}\" ARGS 
	\"${GENERATOR_ARGUMENT}\" \"-S${CMAKE_3RDPARTY_EIGEN_SRC_DIR}\" \"-B${CMAKE_3RDPARTY_EIGEN_BUILD_DIR}\"
	-DBUILD_TESTING=FALSE
	-DCMAKE_INSTALL_PREFIX=\"${3RDPARTY_INSTALL_PREFIX}\"
	-DCMAKE_INSTALL_INCLUDEDIR=\"include\"
	${EXTERNAL_FLAG_SETTER}
	)
message(STATUS "Installing Eigen3")
EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
	--install \"${CMAKE_3RDPARTY_EIGEN_BUILD_DIR}\" --config Release )
message(STATUS "done.")
	