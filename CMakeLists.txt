CMAKE_MINIMUM_REQUIRED(VERSION 3.15)

PROJECT(SensorFusion)

if (WIN32)
	## ---------------------------------------------------------------------------------
	## Use folders in IDEs supporting it
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
	set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "")
	## ---------------------------------------------------------------------------------
	## Set /MT or /MD compiler flags
	set(RUNTIME_LIBRARY_SETTINGS "/MT*" CACHE STRING "Code Generation: multi-threaded (/MT*) or multi threaded DLL (/MD*)")
	set_property(CACHE RUNTIME_LIBRARY_SETTINGS PROPERTY STRINGS "/MT*" "/MD*")
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
	if (RUNTIME_LIBRARY_SETTINGS STREQUAL "/MT*")
		set(REPLACE_ARGS "/MD" "/MT")
		set(RUNTIME_INFO "mt")
	else()
		set(REPLACE_ARGS "/MT" "/MD")
		set(RUNTIME_INFO "md")
	endif()
	foreach(CompilerFlag ${CompilerFlags})
		string(REPLACE ${REPLACE_ARGS} ${CompilerFlag} "${${CompilerFlag}}")
		list(APPEND EXTERNAL_FLAG_SETTER -D${CompilerFlag}="${${CompilerFlag}}" ) #produces a list like -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
	endforeach()
	## ---------------------------------------------------------------------------------
	## Set configurations to be used
	set (CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo CACHE STRING "Configuration types" FORCE)
endif()

## ---------------------------------------------------------------------------------
## Set folders
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "" FORCE) #Nem convention instead of program files/SensorFusion...
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Bin) #Or the program puts the application into different folders...

## ---------------------------------------------------------------------------------
## Draft settings - use SF_BUILD_DRAFT_API in the code for experimental API 
if(EXISTS "${CMAKE_SOURCE_DIR}/.git")
  option(ENABLE_DRAFTS "Build and install draft classes and methods" ON)
else()
  option(ENABLE_DRAFTS "Build and install draft classes and methods" OFF)
endif()
if(ENABLE_DRAFTS)
  add_definitions(-DSF_BUILD_DRAFT_API)
endif()

## ---------------------------------------------------------------------------------
## TODO clean this part...
if(UNIX AND NOT APPLE)
    set(LINUX TRUE)
    #This is required to find sodium package for libzmq linking
    #set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif()
# if(NOT LINUX) should work, too, if you need that
if(LINUX) 
	message(STATUS ">>> Linux")
	set (CMAKE_CXX_STANDARD 14)
elseif(WIN32)
        # for Windows operating system in general
        # on *nixes we need this to use C++14
        if (MSVC_VERSION GREATER_EQUAL "1900")
            include(CheckCXXCompilerFlag)
            CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
            if (_cpp_latest_flag_supported)
                add_compile_options("/std:c++latest")
            endif()
        endif()
endif()

## ---------------------------------------------------------------------------------
## Get generator and platform to use for third party building (e.g. Visual Studio 2017 x64)
set( GENERATOR_ARGUMENT "" CACHE STRING "Used to build third parties")
if (GENERATOR_ARGUMENT STREQUAL "")
	message(STATUS "Getting generator and architecture...")
	include(cmake/GetGeneratorArgument.cmake)
	GetGeneratorArgument(RES)
	message (STATUS "  ...succeded: ${RES}")
	set(GENERATOR_ARGUMENT ${RES} CACHE STRING "Used to build third parties" FORCE)
endif()

## ---------------------------------------------------------------------------------
## Generally used variables for third parties
include(cmake/ThirdPartyHandling.cmake)
set(THIRD_PARTY_CONFIG Debug;Release)
set(ONLY_RELEASE Release)
set(GIT_NAMES GIT Git)
set(HG_NAMES HG Hg)

if (CMAKE_GENERATOR_PLATFORM STREQUAL "Win32")
	message(FATAL_ERROR "Don't use ZMQ on x86 platform!")
endif()
set(ZMQ_COMMAND "git clone https://github.com/zeromq/libzmq.git -bv4.3.2")
set(ZMQ_CACHE_SETTINGS -DBUILD_SHARED=TRUE;-DBUILD_STATIC=TRUE;-DBUILD_TESTS=FALSE;
	-DENABLE_CPACK=FALSE;-DENABLE_DRAFTS=FALSE;-DENABLE_RADIX_TREE:BOOL=TRUE;-DLIBZMQ_PEDANTIC=TRUE;
	-DWITH_PERF_TOOL=FALSE;-DWITH_DOCS=FALSE;-DZMQ_BUILD_TESTS=FALSE;
	-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";-DENABLE_CURVE:BOOL=FALSE)
add_definitions(-DZMQ_STATIC)
if (WIN32)
	list(APPEND ZMQ_CACHE_SETTINGS ${EXTERNAL_FLAG_SETTER})
	add_definitions(-DZMQ_HAVE_WINDOWS)
	if (CMAKE_GENERATOR STREQUAL "Visual Studio 15 2017")
		add_definitions(-DZMQ_USE_CV_IMPL_WIN32API)
		list(APPEND ZMQ_CACHE_SETTINGS -DZMQ_CV_IMPL:STRING=win32api)
	else()
		add_definitions(-DZMQ_USE_CV_IMPL_STL11)
		list(APPEND ZMQ_CACHE_SETTINGS -DZMQ_CV_IMPL:STRING=stl11)
	endif()
else()
	include(CheckCXXSymbolExists)
	check_cxx_symbol_exists(SO_PEERCRED sys/socket.h ZMQ_HAVE_SO_PEERCRED)
	check_cxx_symbol_exists(LOCAL_PEERCRED sys/socket.h ZMQ_HAVE_LOCAL_PEERCRED)
	if (ZMQ_HAVE_SO_PEERCRED)
		add_definitions(-DZMQ_HAVE_SO_PEERCRED)
	endif()
	if (ZMQ_HAVE_LOCAL_PEERCRED)
		add_definitions(-DZMQ_HAVE_LOCAL_PEERCRED)
	endif()
endif()
if (UNIX)
	add_definitions(-DZMQ_HAVE_LINUX)
	add_definitions(-DZMQ_HAVE_TIPC)
	list(APPEND ZMQ_CACHE_SETTINGS -DZMQ_CV_IMPL:STRING=pthreads)
endif()
third_party_policy(ZeroMQ ${ZMQ_COMMAND} ZMQ_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
if(ZeroMQ_FOUND)
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
endif()
## ---------------------------------------------------------------------------------
## Find or get CPPZMQ
set(CPPZMQ_COMMAND "git clone https://github.com/zeromq/cppzmq.git -bv4.5.0")
set(CPPZMQ_CACHE_SETTINGS -DENABLE_DRAFTS=FALSE;-DBUILD_TESTING=FALSE;-DCPPZMQ_BUILD_TESTS=FALSE;
	-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";${EXTERNAL_FLAG_SETTER})
third_party_policy(cppzmq ${CPPZMQ_COMMAND} CPPZMQ_CACHE_SETTINGS ONLY_RELEASE false GIT_NAMES)
macro(requires_cppzmq NAME)
	target_link_libraries (${NAME} cppzmq-static)
endmacro(requires_cppzmq)
## ---------------------------------------------------------------------------------
## Find or get FlatBuffers
set(FLATBUFFERS_COMMAND "git clone https://github.com/google/flatbuffers.git -bv1.11.0")
set(FLATBUFFERS_CACHE_SETTINGS -DFLATBUFFERS_BUILD_FLATC=FALSE;-DFLATBUFFERS_BUILD_FLATHASH=FALSE;
	-DFLATBUFFERS_BUILD_FLATLIB=FALSE;-DFLATBUFFERS_BUILD_TESTS=FALSE;-DFLATBUFFERS_LIBXX_WITH_CLANG=FALSE;
	-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";${EXTERNAL_FLAG_SETTER})
third_party_policy(Flatbuffers ${FLATBUFFERS_COMMAND} FLATBUFFERS_CACHE_SETTINGS ONLY_RELEASE false GIT_NAMES)
if (Flatbuffers_FOUND)
	find_path(FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h PATHS "${3RDPARTY_INSTALL_PREFIX}/include")
endif()
macro(requires_flatbuffers)
	include_directories(${FLATBUFFERS_INCLUDE_DIR}) #targetinclude_directory????
endmacro(requires_flatbuffers)
## ---------------------------------------------------------------------------------
## Find or get SPDLOG
set(SPDLOG_COMMAND "git clone https://github.com/gabime/spdlog.git -bv1.5.0")
set(SPDLOG_CACHE_SETTINGS -DSPDLOG_BUILD_BENCH=FALSE;-DSPDLOG_BUILD_EXAMPLE=FALSE;
	-DSPDLOG_BUILD_EXAMPLE_HO=FALSE;-DSPDLOG_BUILD_TESTS=FALSE;-DSPDLOG_BUILD_TESTS_HO=FALSE;
	-DSPDLOG_INSTALL=TRUE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";${EXTERNAL_FLAG_SETTER})
third_party_policy(spdlog ${SPDLOG_COMMAND} SPDLOG_CACHE_SETTINGS THIRD_PARTY_CONFIG TRUE GIT_NAMES)
macro(requires_spdlog NAME)
	target_link_libraries (${NAME} spdlog::spdlog)
endmacro(requires_spdlog)
## ---------------------------------------------------------------------------------
## Find or get Eigen
set(EIGEN3_COMMAND "hg clone --verbose https://bitbucket.org/eigen/eigen -r3.3.7")
set(EIGEN3_CACHE_SETTINGS -DBUILD_TESTING=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";
	-DCMAKE_INSTALL_INCLUDEDIR="include";${EXTERNAL_FLAG_SETTER})
third_party_policy(Eigen3 ${EIGEN3_COMMAND} EIGEN3_CACHE_SETTINGS ONLY_RELEASE false HG_NAMES)
macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

option(ADD_PLOTTER "To use OpenCV-based plotters" FALSE)
if (ADD_PLOTTER)
	## ---------------------------------------------------------------------------------
	## Find or OpenCV
	if (RUNTIME_LIBRARY_SETTINGS STREQUAL "/MT*")
		set(BUILD_OPENCV_WITH_STATIC_CRT true)
	else()
		set(BUILD_OPENCV_WITH_STATIC_CRT false)
	endif()
	set(OpenCV_STATIC TRUE)
	set(OPENCV_COMMAND "git clone https://github.com/opencv/opencv.git -b4.1.2")
	set(OPENCV_CACHE_SETTINGS -DBUILD_opencv_apps=FALSE;-DBUILD_opencv_calib3d=FALSE;-DBUILD_opencv_dnn=FALSE;
		-DBUILD_opencv_features2d=FALSE;-DBUILD_opencv_flann=FALSE;-DBUILD_opencv_gapi=FALSE;-DBUILD_opencv_java_bindings_generator=FALSE;
		-DBUILD_opencv_ml=FALSE;-DBUILD_opencv_objdetect=FALSE;-DBUILD_opencv_photo=FALSE;-DBUILD_opencv_python_bindings_generator=FALSE;
		-DBUILD_opencv_python_tests=FALSE;-DBUILD_opencv_stiching=FALSE;-DBUILD_opencv_ts=FALSE;-DBUILD_opencv_video=FALSE;
		-DBUILD_opencv_videoio=FALSE;-DBUILD_opencv_apps=FALSE;-DBUILD_JAVA=FALSE;-DBUILD_OPENEXR=FALSE;-DBUILD_PACKAGE=FALSE;
		-DBUILD_PERF_TESTS=FALSE;-DBUILD_PROTOBUF=FALSE;-DBUILD_SHARED_LIBS=FALSE;-DBUILD_WITH_STATIC_CRT=${BUILD_OPENCV_WITH_STATIC_CRT};
		-DBUILD_TESTS=FALSE;-DBUILD_TIFF=FALSE;-DWITH_ADE=FALSE;-DWITH_FFMPEG=FALSE;-DWITH_EIGEN=FALSE;-DCPU_DISPATCH="";
		-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";${EXTERNAL_FLAG_SETTER})
	third_party_policy(OpenCV ${OPENCV_COMMAND} OPENCV_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
	if(OpenCV_FOUND)
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
	endif()
	macro(requires_opencv NAME)
		target_link_libraries ( ${NAME} opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc ) # or ${OpenCV_LIBS}
	endmacro(requires_opencv)
endif(ADD_PLOTTER)
## ---------------------------------------------------------------------------------
## Names used in the CMake package
set(export_config_name "SensorFusion")
set(export_targets ${export_config_name}Targets)
if(WIN32)
	set(export_targets_name ${export_config_name}Targets-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO})
else()
	set(export_targets_name ${export_config_name}Targets)
endif()

## ---------------------------------------------------------------------------------
## Macro to setup archive targets
macro(install_lib NAME HEADERS)
	set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL} "${NAME}" PARENT_SCOPE) #for configuring Config.cmake in installing.cmake
	set_target_properties(${NAME} PROPERTIES FOLDER "Lib" IMPORTED_LOCATION lib)
	if(WIN32)
		set_target_properties(${NAME} PROPERTIES
			RELEASE_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}"
			RELWITHDEBINFO_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-relwithdebinfo"
			MINSIZEREL_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-minsizerel"
			DEBUG_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-debug")
	endif()
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

## ---------------------------------------------------------------------------------
## Get the content of subfolders
add_subdirectory (types)
if (ADD_PLOTTER)
	add_subdirectory (plotter)
	add_subdirectory (plotter_test)
endif()
add_subdirectory (core)
add_subdirectory (communication)

## ---------------------------------------------------------------------------------
## Target to clean install folder
add_custom_target(uninstall COMMAND  ${CMAKE_COMMAND} -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} -P "${CMAKE_SOURCE_DIR}/cmake/CleanInstall.cmake"  )

## ---------------------------------------------------------------------------------
## Additional install methods
install(EXPORT ${export_targets}
  FILE ${export_targets_name}.cmake
  DESTINATION "cmake"
  NAMESPACE ${export_config_name}::)
# Because git version cannot be returned in build-time, the install goes via install(CODE *)...
if(WIN32)
	install(CODE "
		include(\"${CMAKE_SOURCE_DIR}/cmake/GetGitVersion.cmake\")

		set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL})
		set(RUNTIME_INFO ${RUNTIME_INFO})
		set(export_config_name ${export_config_name})
		set(export_targets ${export_targets})

		#----------
		# Create a script to check the current export to the latest *Version.cmake
		file(WRITE \"${CMAKE_INSTALL_PREFIX}/cmake/${export_config_name}VersionCheck-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-\${BUILD_TYPE}.cmake\" \"
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

		set(ADD_PLOTTER ${ADD_PLOTTER})
		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/ConfigWindows.cmake.in\"
		\"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake\" @ONLY)
	")
else()
	install(CODE "
		include(\"${CMAKE_SOURCE_DIR}/cmake/GetGitVersion.cmake\")

		set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL})
		set(export_config_name ${export_config_name})
		set(export_targets ${export_targets})
		set(export_targets_name ${export_targets_name})

		#------------------------------------------------------------------------------
		# Configure <export_config_name>ConfigVersion.cmake common to build and install tree
		include(CMakePackageConfigHelpers)
		write_basic_package_version_file(
			\"${CMAKE_INSTALL_PREFIX}/${export_config_name}ConfigVersion.cmake\"
			VERSION \${GIT_TAG_VERSION}
			COMPATIBILITY ExactVersion)

		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/sf_version.h.in\"
			\"${CMAKE_INSTALL_PREFIX}/include/sf_version.h\" @ONLY)

		set(ADD_PLOTTER ${ADD_PLOTTER})
		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/ConfigLinux.cmake.in\"
		\"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake\" @ONLY)
	")
endif()

## ---------------------------------------------------------------------------------
## To generate doxygen documentation automatically during install
option(WITH_DOCS "To generate documentation into the package during install (neeeds doxygen and latex - MikTex)" false)
if(WITH_DOCS)
	configure_file("${CMAKE_CURRENT_LIST_DIR}/cmake/Docs/Doxyfile.in" "${CMAKE_BINARY_DIR}/Doxyfile" @ONLY)
	find_program(DOXYGEN doxygen)
	find_program(LATEX latex)
	install(CODE "EXEC_PROGRAM(doxygen \"${CMAKE_BINARY_DIR}/Doxyfile\")
		file(COPY \"${CMAKE_SOURCE_DIR}/cmake/docs/index.html\" DESTINATION \"${CMAKE_INSTALL_PREFIX}/Docs\" )
	" )
endif()

## ---------------------------------------------------------------------------------
## Tests
option(BUILD_TESTING "To build the unit tests" TRUE)
if (BUILD_TESTING)
	enable_testing()
	add_subdirectory(tests)

endif()

#readme: tested on:
#versions, versions of third parties...

#TODO: update package title page, pictures...