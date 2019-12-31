CMAKE_MINIMUM_REQUIRED(VERSION 3.2) # is enough??

PROJECT(SensorFusion)

# use folders in IDEs supporting it
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "")

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
set(EXTERNAL_FLAG_SETTER "")
foreach(CompilerFlag ${CompilerFlags})
  string(REPLACE "/MD" "/MT" ${CompilerFlag} "${${CompilerFlag}}")
  set(EXTERNAL_FLAG_SETTER "${EXTERNAL_FLAG_SETTER} -D${CompilerFlag}=\"${${CompilerFlag}}\"" ) #produces a list like -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
endforeach()

# where to put the resulting binaries?
set (APPLICATION Bin)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${APPLICATION})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${APPLICATION})
#####################################################################################
set (CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo CACHE STRING "Configuration types" FORCE)


## Set folders
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "" FORCE)

#####################################################################################
#####################Minden kell ebbõl?##############################################

#


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

set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR}" CACHE STRING "")
if (${GENERATOR_ARGUMENT} STREQUAL "")
	message(STATUS "Getting generator and architecture...")
	include(cmake/GetGeneratorArgument.cmake)
	GetGeneratorArgument(RES)
	message (STATUS "  ...succeded: ${RES}")
	set(GENERATOR_ARGUMENT ${RES} CACHE STRING "Used to build third parties" FORCE)
endif()

# TODO: add gtest, linux compatibility?

set (CMAKE_3RDPARTY_DIR "${CMAKE_BINARY_DIR}/Third parties")
file(MAKE_DIRECTORY ${CMAKE_3RDPARTY_DIR})
#Policy_all_thirdparties
set(Policy_ALL_3RD_PARTIES "Setting one by one" CACHE STRING "How to get 3rdParties")
set_property(CACHE Policy_ALL_3RD_PARTIES PROPERTY STRINGS "Setting one by one" "Download all")

### ZMQ
set(Policy_ZMQ "Choose" CACHE STRING "How to get ZeroMQ")
set_property(CACHE Policy_ZMQ PROPERTY STRINGS "Choose" "Search on the path" "Download")
if (Policy_ZMQ STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
	find_package(ZeroMQ QUIET PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	if (NOT ZeroMQ_FOUND)
		include(cmake/DownloadAndInstallZeroMQ.cmake)
		find_package(ZeroMQ REQUIRED PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	endif()
elseif (Policy_ZMQ STREQUAL  "Search on the path")
	find_package(ZeroMQ REQUIRED)
else()
	message( SEND_ERROR "Policy for ZeroMQ must be chosen!" )
endif()

### CPPZMQ
set(Policy_CPPZMQ "Choose" CACHE STRING "How to get CPPZMQ")
set_property(CACHE Policy_CPPZMQ PROPERTY STRINGS "Choose" "Search on the path" "Download")
if (Policy_CPPZMQ STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
	find_package(CPPZMQ QUIET PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	if (NOT CPPZMQ_FOUND)
		include(cmake/DownloadAndInstallCPPZMQ.cmake)
		find_package(CPPZMQ REQUIRED PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	endif()
elseif (Policy_CPPZMQ STREQUAL  "Search on the path")
	find_package(CPPZMQ REQUIRED)
else()
	message( SEND_ERROR "Policy for CPPZMQ must be chosen!" )
endif()

macro(requires_cppzmq NAME)
	target_link_libraries (${NAME} cppzmq-static)
	add_definitions(-DZMQ_STATIC)
endmacro(requires_cppzmq)

### FlatBuffers
set(Policy_FlatBuffers "Choose" CACHE STRING "How to get FlatBuffers")
set_property(CACHE Policy_FlatBuffers PROPERTY STRINGS "Choose" "Search on the path" "Download")
if (Policy_FlatBuffers STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
	find_package(FlatBuffers QUIET PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	if (NOT FlatBuffers_FOUND)
		include(cmake/DownloadAndInstallFlatBuffers.cmake)
		find_package(FlatBuffers REQUIRED PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	endif()
elseif (Policy_FlatBuffers STREQUAL  "Search on the path")
	find_package(FlatBuffers REQUIRED)
else()
	message( SEND_ERROR "Policy for FlatBuffers must be chosen!" )
endif()
find_path(FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h)

macro(requires_flatbuffers)
	include_directories(${FLATBUFFERS_INCLUDE_DIR})
endmacro(requires_flatbuffers)

### Eigen3
set(Policy_Eigen3 "Choose" CACHE STRING "How to get Eigen3")
set_property(CACHE Policy_Eigen3 PROPERTY STRINGS "Choose" "Search on the path" "Download")
if (Policy_Eigen3 STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
	find_package(Eigen3 3.3 QUIET NO_MODULE PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	if (NOT Eigen3_FOUND)
		include(cmake/DownloadAndInstallEigen.cmake)
		find_package(Eigen3 3.3 REQUIRED NO_MODULE PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	endif()
elseif (Policy_Eigen3 STREQUAL  "Search on the path")
	find_package(Eigen3 3.3 REQUIRED NO_MODULE)
else()
	message( SEND_ERROR "Policy for Eigen3 must be chosen!" )
endif()

macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

### OpenCV
set(OpenCV_STATIC TRUE)
set(Policy_OpenCV "Choose" CACHE STRING "How to get OpenCV")
set_property(CACHE Policy_OpenCV PROPERTY STRINGS "Choose" "Search on the path" "Download")
if (Policy_OpenCV STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
	find_package(OpenCV QUIET PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	if (NOT OpenCV_FOUND)
		include(cmake/DownloadAndInstallOpenCV.cmake)
		find_package(OpenCV REQUIRED PATHS ${CMAKE_INSTALL_PREFIX} NO_DEFAULT_PATH )
	endif()
elseif (Policy_OpenCV STREQUAL  "Search on the path")
	find_package(OpenCV REQUIRED)
else()
	message( SEND_ERROR "Policy for OpenCV must be chosen!" )
endif()

macro(requires_opencv NAME)
	include_directories(${OpenCV_INCLUDE_DIRS})
	target_link_libraries ( ${NAME} ${OpenCV_LIBS} )
endmacro(requires_opencv)
if (0)
macro(requires_opencv NAME)
	include_directories( #${OpenCV_INCLUDE_DIRS} 
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/core/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/highgui/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/imgproc/include
		${CMAKE_3RDPARTY_OPENCV_SRC_DIR}/modules/imgcodecs/include
		${CMAKE_BINARY_DIR}
		)
	target_link_libraries ( ${NAME} opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs) #${opencv_libs}
endmacro(requires_opencv)
endif()

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



#TODO: set include folders to targets, install!!!