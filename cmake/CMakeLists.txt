PROJECT(SensorFusionExternals)

CMAKE_MINIMUM_REQUIRED(VERSION 3.10) # TODO check...

#####################Minden kell ebbõl???##############################################
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

set (CMAKE_CONFIGURATION_TYPES Debug;Release CACHE STRING "Configuration types" FORCE)


## Generate Compiler options
include (CompilerFlags.cmake)

############################################################################################################
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
set(EXTERNAL_FLAG_SETTER "")
foreach(CompilerFlag ${CompilerFlags})
  set(EXTERNAL_FLAG_SETTER ${EXTERNAL_FLAG_SETTER} "-D${CompilerFlag}:STRING=${${CompilerFlag}}" ) #produces a list like -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
endforeach()

############################################################################################################
# Add dependencies as external libraries
############################################################################################################

set(DEPENDENCIES_DIR ${CMAKE_BINARY_DIR})

include(ExternalProject)

############################################################################################################
# Add Eigen3 (3.3.7) to the project as "Eigen3", with include path ${EIGEN3_INCLUDE_DIRS}, TODO: targetdefinitions!

ExternalProject_Add(Ext-Eigen3
			HG_REPOSITORY https://bitbucket.org/eigen/eigen
			HG_TAG 3.3.7
			UPDATE_COMMAND ""
			PATCH_COMMAND ""
			BUILD_COMMAND ""
			INSTALL_COMMAND ""
			LOG_DOWNLOAD ON
			LOG_CONFIGURE ON
		PREFIX ${DEPENDENCIES_DIR}
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
)
set(EIGEN3_INCLUDE_DIRS ${DEPENDENCIES_DIR}/src/Ext-Eigen3 )

############################################################################################################
# Add zmqlib (v4.3.2) to the project as "zmqlib", with include path ${ZMQLIB_INCLUDE_DIR}

message(STATUS ${CMAKE_COMMAND} "<-")

ExternalProject_Add(Ext-zmqlib
				GIT_REPOSITORY https://github.com/zeromq/libzmq.git
				GIT_TAG v4.3.2
				GIT_PROGRESS TRUE
				UPDATE_COMMAND ""
				PATCH_COMMAND ""
				#BUILD_COMMAND ""
				INSTALL_COMMAND ""
				LOG_DOWNLOAD ON
				LOG_CONFIGURE ON
				LOG_BUILD ON
				LOG_INSTALL ON
				PREFIX ${DEPENDENCIES_DIR}
				CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
					-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
				CMAKE_CACHE_ARGS -DBUILD_SHARED:BOOL=TRUE #
					-DBUILD_STATIC:BOOL=TRUE
					-DBUILD_TESTS:BOOL=FALSE
					-DENABLE_CPACK:BOOL=FALSE
					-DENABLE_CURVE:BOOL=FALSE
					-DENABLE_DRAFTS:BOOL=FALSE
					-DENABLE_RADIXTREE:BOOL=FALSE
					-DLIBZMQ_PEDANTIC:BOOL=FALSE
					-DWITH_PERF_TOOL:BOOL=FALSE
					-DZMQ_BUILD_TESTS:BOOL=FALSE
					${EXTERNAL_FLAG_SETTER}
)

set( ZMQ_SRC_DIR ${DEPENDENCIES_DIR}/src/Ext-zmqlib )
set( ZMQ_BUILD_DIR ${DEPENDENCIES_DIR}/src/Ext-zmqlib-build )
set( ZMQLIB_INCLUDE_DIRS ${ZMQ_SRC_DIR}/ ${ZMQ_SRC_DIR}/include)

############################################################################################################
# Add cppzmq (v4.5.0) to the project

ExternalProject_Add(Ext-cppzmq
			DEPENDS Ext-zmqlib
			GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
			GIT_TAG v4.5.0
			GIT_PROGRESS TRUE
			UPDATE_COMMAND ""
			PATCH_COMMAND ""
			#BUILD_COMMAND ""
			INSTALL_COMMAND ""
			LOG_DOWNLOAD ON
			LOG_CONFIGURE ON
			LOG_BUILD ON
			LOG_INSTALL ON
			PREFIX ${DEPENDENCIES_DIR}
			CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
				-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
			CMAKE_CACHE_ARGS -DBUILD_TESTING:BOOL=FALSE
				-DCPPZMQ_BUILD_TESTS:BOOL=FALSE
				-DENABLE_DRAFTS:BOOL=FALSE
				-DZeroMQ_DIR:STRING=${ZMQ_BUILD_DIR}
				-DZeroMQ_LIBRARY:STRING=${ZMQ_BUILD_DIR}/bin
				-DZeroMQ_STATIC_LIBRARY:STRING=${ZMQ_BUILD_DIR}/lib
				${EXTERNAL_FLAG_SETTER}
)

ExternalProject_Add(Ext-flatbuffers
				GIT_REPOSITORY https://github.com/google/flatbuffers.git
				GIT_TAG v1.11.0
				GIT_PROGRESS TRUE
				UPDATE_COMMAND ""
				PATCH_COMMAND ""
				BUILD_COMMAND ""
				INSTALL_COMMAND ""
				LOG_DOWNLOAD ON
				LOG_CONFIGURE ON
				LOG_BUILD ON
				LOG_INSTALL ON
				BUILD_IN_SOURCE ON
				PREFIX ${DEPENDENCIES_DIR}
				CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
					-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
				CMAKE_CACHE_ARGS
					-DFLATBUFFERS_BUILD_FLATC:BOOL=TRUE #
					-DFLATBUFFERS_BUILD_FLATHASH:BOOL=TRUE
					-DFLATBUFFERS_BUILD_FLATLIB:BOOL=TRUE
					-DFLATBUFFERS_BUILD_GRPCTEST:BOOL=FALSE
					-DFLATBUFFERS_BUILD_SHAREDLIB:BOOL=FALSE
					-DFLATBUFFERS_BUILD_TESTS:BOOL=FALSE
					-DFLATBUFFERS_INSTALL:BOOL=TRUE
					-DFLATBUFFERS_LIBCXX_WITH_CLANG:BOOL=TRUE
					-DFLATBUFFERS_PACKAGE_DEBIAN:BOOL=FALSE
					-DFLATBUFFERS_PACKAGE_REDHAT:BOOL=FALSE
					${EXTERNAL_FLAG_SETTER}
)

ExternalProject_Add(Ext-opencv
				GIT_REPOSITORY https://github.com/opencv/opencv.git
				GIT_TAG 4.1.2
				GIT_PROGRESS TRUE
				UPDATE_COMMAND ""
				PATCH_COMMAND ""
				#BUILD_COMMAND ""
				INSTALL_COMMAND ""
				LOG_DOWNLOAD ON
				LOG_CONFIGURE ON
				LOG_BUILD ON
				LOG_INSTALL ON
				#BUILD_IN_SOURCE ON
				PREFIX ${DEPENDENCIES_DIR}
				CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
					-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
				CMAKE_CACHE_ARGS
					-DBUILD_opencv_apps:BOOL=FALSE
					-DBUILD_opencv_calib3d:BOOL=FALSE
					-DBUILD_opencv_dnn:BOOL=FALSE
					-DBUILD_opencv_features2d:BOOL=FALSE
					-DBUILD_opencv_flann:BOOL=FALSE
					-DBUILD_opencv_gapi:BOOL=FALSE
					-DBUILD_opencv_java_bindings_generator:BOOL=FALSE
					-DBUILD_opencv_ml:BOOL=FALSE
					-DBUILD_opencv_objdetect:BOOL=FALSE
					-DBUILD_opencv_photo:BOOL=FALSE
					-DBUILD_opencv_python_bindings_generator:BOOL=FALSE
					-DBUILD_opencv_python_tests:BOOL=FALSE
					-DBUILD_opencv_stiching:BOOL=FALSE
					-DBUILD_opencv_ts:BOOL=FALSE
					-DBUILD_opencv_video:BOOL=FALSE
					-DBUILD_opencv_videoio:BOOL=FALSE
					-DBUILD_opencv_apps:BOOL=FALSE
					-DBUILD_JAVA:BOOL=FALSE
					-DBUILD_OPENEXR:BOOL=FALSE
					-DBUILD_PACKAGE:BOOL=FALSE
					-DBUILD_PERF_TESTS:BOOL=FALSE
					-DBUILD_PROTOBUF:BOOL=FALSE
					-DBUILD_SHARED_LIBS:BOOL=FALSE
					-DBUILD_TESTS:BOOL=FALSE
					-DBUILD_TIFF:BOOL=FALSE
					${EXTERNAL_FLAG_SETTER}
)
#TODO: gtest