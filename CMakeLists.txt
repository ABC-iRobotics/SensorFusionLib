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
		CMAKE_C_FLAGS_MINSIZEREL)
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


## Set folders - TODO: clear from the final version!!
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "" FORCE)

#####################################################################################
#####################Minden kell ebbõl?##############################################


option(ADD_PLOTTER "To use OpenCV-based plotters" TRUE)

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

set( GENERATOR_ARGUMENT "" CACHE STRING "Used to build third parties")
if (GENERATOR_ARGUMENT STREQUAL "")
	message(STATUS "Getting generator and architecture...")
	include(cmake/GetGeneratorArgument.cmake)
	GetGeneratorArgument(RES)
	message (STATUS "  ...succeded: ${RES}")
	set(GENERATOR_ARGUMENT ${RES} CACHE STRING "Used to build third parties" FORCE)
endif()

# TODO: add gtest, linux compatibility?

set (CMAKE_3RDPARTY_DIR "${CMAKE_BINARY_DIR}/Third parties")
set (3RDPARTY_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/Third parties")
file(MAKE_DIRECTORY ${CMAKE_3RDPARTY_DIR})
#Policy_all_thirdparties
set(Policy_ALL_3RD_PARTIES "Setting one by one" CACHE STRING "How to get 3rdParties")
set_property(CACHE Policy_ALL_3RD_PARTIES PROPERTY STRINGS "Setting one by one" "Download all")

macro(third_party_policy NAME COMMAND CACHE_LIST_NAME CONFIG_LIST_NAME DOES_NEED_BUILDING)
	set(Policy_${NAME} "Choose" CACHE STRING "How to get ${NAME}")
	set_property(CACHE Policy_${NAME} PROPERTY STRINGS "Choose" "Search on the path" "Download")
	if (Policy_${NAME} STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
		find_package(${NAME} QUIET PATHS ${3RDPARTY_INSTALL_PREFIX} NO_DEFAULT_PATH )
		if (NOT ${NAME}_FOUND)
			set (DIR "${CMAKE_3RDPARTY_DIR}/${NAME}")
			set (SRC_DIR "${DIR}/src")
			set (BUILD_DIR "${DIR}/build")
			# CLONE
			message( STATUS "Downloading ${NAME} (to ${SRC_DIR})...")
			EXEC_PROGRAM( "${COMMAND} \"${SRC_DIR}\"" )
			message ( STATUS " succeeded. CMake configure started....")
			# Call configure, generate and build (Release & Debug) on the externals
			message(STATUS "Generating makefile/solution for ${NAME}")
			EXEC_PROGRAM(\"${CMAKE_COMMAND}\" \"${SRC_DIR}\" ARGS \"${GENERATOR_ARGUMENT}\" \"-S${SRC_DIR}\"
				\"-B${BUILD_DIR}\" ${${CACHE_LIST_NAME}} ${EXTERNAL_FLAG_SETTER} )
			foreach(config ${${CONFIG_LIST_NAME}})
				if(${DOES_NEED_BUILDING})
					message(STATUS "Building ${NAME} (${config}). It can take some time...")
					EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
						--build \"${BUILD_DIR}\" --config ${config} )
				endif()
				message(STATUS "Installing ${NAME} (${config}).")
				EXEC_PROGRAM(${CMAKE_COMMAND} ARGS
					--install \"${BUILD_DIR}\" --config ${config} )
			endforeach()
			message(STATUS "done.")
			find_package(${NAME} REQUIRED PATHS ${3RDPARTY_INSTALL_PREFIX} NO_DEFAULT_PATH )
		endif()
	elseif (Policy_${NAME} STREQUAL  "Search on the path")
		find_package(${NAME} REQUIRED)
	else()
		message( SEND_ERROR "Policy for ${NAME} must be chosen!" )
	endif()
endmacro(third_party_policy)

set(THIRD_PARTY_CONFIG Debug;Release)
set(ONLY_RELEASE Release)

### ZeroMQ
set(ZMQ_COMMAND "git clone https://github.com/zeromq/libzmq.git -bv4.3.2")
set(ZMQ_CACHE_SETTINGS -DBUILD_SHARED=TRUE;-DBUILD_STATIC=TRUE;-DBUILD_TESTS=FALSE;-DENABLE_CPACK=FALSE;
	-DENABLE_CURVE=FALSE;-DENABLE_DRAFTS=FALSE;-DENABLE_RADIX_TREE=FALSE;-DLIBZMQ_PEDANTIC=FALSE;
	-DWITH_PERF_TOOL=FALSE;-DWITH_DOCS=FALSE;-DZMQ_BUILD_TESTS=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")
third_party_policy(ZeroMQ ${ZMQ_COMMAND} ZMQ_CACHE_SETTINGS THIRD_PARTY_CONFIG true)
if(ZeroMQ_FOUND)
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
endif()
### CPPZMQ
set(CPPZMQ_COMMAND "git clone https://github.com/zeromq/cppzmq.git -bv4.5.0")
set(CPPZMQ_CACHE_SETTINGS -DENABLE_DRAFTS=FALSE;-DBUILD_TESTING=FALSE;-DCPPZMQ_BUILD_TESTS=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")
third_party_policy(CPPZMQ ${CPPZMQ_COMMAND} CPPZMQ_CACHE_SETTINGS ONLY_RELEASE false)
macro(requires_cppzmq NAME)
	target_link_libraries (${NAME} cppzmq-static)
	add_definitions(-DZMQ_STATIC)
endmacro(requires_cppzmq)
### FlatBuffers
set(FLATBUFFERS_COMMAND "git clone https://github.com/google/flatbuffers.git -bv1.11.0")
set(FLATBUFFERS_CACHE_SETTINGS -DFLATBUFFERS_BUILD_FLATC=FALSE;-DFLATBUFFERS_BUILD_FLATHASH=FALSE;
	-DFLATBUFFERS_BUILD_FLATLIB=FALSE;-DFLATBUFFERS_BUILD_TESTS=FALSE;-DFLATBUFFERS_LIBXX_WITH_CLANG=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")
third_party_policy(FlatBuffers ${FLATBUFFERS_COMMAND} FLATBUFFERS_CACHE_SETTINGS ONLY_RELEASE false)
if (FlatBuffers_FOUND)
	find_path(FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h PATHS "${3RDPARTY_INSTALL_PREFIX}/include")
endif()
macro(requires_flatbuffers)
	include_directories(${FLATBUFFERS_INCLUDE_DIR}) #targetinclude_directory????
endmacro(requires_flatbuffers)
### Eigen
set(EIGEN3_COMMAND "hg clone --verbose https://bitbucket.org/eigen/eigen -r3.3.7")
set(EIGEN3_CACHE_SETTINGS -DBUILD_TESTING=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";-DCMAKE_INSTALL_INCLUDEDIR="include")
third_party_policy(Eigen3 ${EIGEN3_COMMAND} EIGEN3_CACHE_SETTINGS ONLY_RELEASE false)
macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

if (ADD_PLOTTER)
	### OpenCV
	set(OpenCV_STATIC TRUE)
	set(OPENCV_COMMAND "git clone https://github.com/opencv/opencv.git -b4.1.2")
	set(OPENCV_CACHE_SETTINGS -DBUILD_opencv_apps=FALSE;-DBUILD_opencv_calib3d=FALSE;-DBUILD_opencv_dnn=FALSE;
		-DBUILD_opencv_features2d=FALSE;-DBUILD_opencv_flann=FALSE;-DBUILD_opencv_gapi=FALSE;-DBUILD_opencv_java_bindings_generator=FALSE;
		-DBUILD_opencv_ml=FALSE;-DBUILD_opencv_objdetect=FALSE;-DBUILD_opencv_photo=FALSE;-DBUILD_opencv_python_bindings_generator=FALSE;
		-DBUILD_opencv_python_tests=FALSE;-DBUILD_opencv_stiching=FALSE;-DBUILD_opencv_ts=FALSE;-DBUILD_opencv_video=FALSE;
		-DBUILD_opencv_videoio=FALSE;-DBUILD_opencv_apps=FALSE;-DBUILD_JAVA=FALSE;-DBUILD_OPENEXR=FALSE;-DBUILD_PACKAGE=FALSE;
		-DBUILD_PERF_TESTS=FALSE;-DBUILD_PROTOBUF=FALSE;-DBUILD_SHARED_LIBS=FALSE;-DBUILD_TESTS=FALSE;-DBUILD_TIFF=FALSE;-DWITH_ADE=FALSE;
		-DWITH_FFMPEG=FALSE;-DWITH_EIGEN=FALSE;-DCPU_DISPATCH="";-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")
	third_party_policy(OpenCV ${OPENCV_COMMAND} OPENCV_CACHE_SETTINGS THIRD_PARTY_CONFIG true)
	if(OpenCV_FOUND)
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
	endif()
	macro(requires_opencv NAME)
		target_link_libraries ( ${NAME} opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc ) # or ${OpenCV_LIBS}
	endmacro(requires_opencv)
endif(ADD_PLOTTER)

set(export_config_name "SensorFusion")
set(export_targets ${export_config_name}Targets)
set(export_targets_name ${export_config_name}Targets-${CMAKE_GENERATOR_PLATFORM})

#### Lib settings
macro(install_lib NAME HEADERS)
	set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL} "${NAME}" PARENT_SCOPE) #for configuring Config.cmake in installing.cmake
	set_target_properties(${NAME} PROPERTIES FOLDER "Lib"
			RELEASE_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-mt"
			RELWITHDEBINFO_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-mt-relwithdebinfo"
			MINSIZEREL_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-mt-minsizerel"
			DEBUG_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-mt-debug"
			IMPORTED_LOCATION lib)
	target_include_directories(${NAME} INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
		$<INSTALL_INTERFACE:include>)
	install(TARGETS ${NAME} EXPORT ${export_targets}
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include)
	install(FILES ${HEADERS} DESTINATION include)
endmacro(install_lib)

add_subdirectory (types)
if (ADD_PLOTTER)
	add_subdirectory (plotter)
	add_subdirectory (plotter_test)
endif()
add_subdirectory (core)
add_subdirectory (communication)
add_subdirectory (ZMQSub_test)
add_subdirectory (ZMQPub_test)

install(EXPORT ${export_targets}
  FILE ${export_targets_name}.cmake
  DESTINATION "cmake"
  NAMESPACE ${export_config_name}::)

# Because git version cannot be returned in build-time, the install goes via script...
install(CODE "
	include(\"${CMAKE_SOURCE_DIR}/cmake/GetGitVersion.cmake\")

	set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL})
	set(export_config_name ${export_config_name})
	set(export_targets ${export_targets})

	#----------
	# Create a script to check the current export to the latest *Version.cmake
	file(WRITE \"${CMAKE_INSTALL_PREFIX}/cmake/${export_config_name}VersionCheck-${CMAKE_GENERATOR_PLATFORM}-\${BUILD_TYPE}.cmake\" \"
	if(NOT SF_VERSION_FULL STREQUAL \\\"\${GIT_VERSION_FULL}\\\")
		message(FATAL_ERROR \\\"${export_config_name} package: wrong \${BUILD_TYPE} version\\\")
	endif()\")

	#------------------------------------------------------------------------------
	# Configure <export_config_name>ConfigVersion.cmake common to build and install tree
	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
		\"${CMAKE_INSTALL_PREFIX}/${export_config_name}ConfigVersion.cmake\"
		VERSION \${GIT_TAG_VERSION}
		COMPATIBILITY ExactVersion)

	configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/sf_version.h.in\"
		\"${CMAKE_INSTALL_PREFIX}/include/sf_version.h\" @ONLY)

	configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/Config.cmake.in\"
	\"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake\" @ONLY)
")
#find_program(GIT git)
#find_program(GIT git) HG!!

#•todo: check cross compile...
#linux, ros compatibility?

#readme: tested on:
#install readme: versions, versions of third parties...
#needs: git, hg, doxygen, latex

#introduce drafts

#doxygen generation! - WITH_DOCS CACHE entry... - for install!!

#◘third party install csak az install paranccsal?

#Googletests - call on third parties??

#választható legyen mt/md flag...

#third party mappaszerkezet: sg/src /build /install
#installra ujabb install parancsok

#tesztelni külön telepített third partykkal

#uninstall custom_command