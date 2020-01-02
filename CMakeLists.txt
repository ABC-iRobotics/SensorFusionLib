CMAKE_MINIMUM_REQUIRED(VERSION 3.2) # is enough??

#Set SF version for library - it will be checked, but cannot be updated manually
set(SF_VERSION 0.0.1)
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" SF_VERSION_MAJOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+)\\..*" "\\1" SF_VERSION_MINOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)-?.*" "\\1" SF_VERSION_PATCH "${SF_VERSION}")
PROJECT(SensorFusion VERSION ${SF_VERSION})

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

macro(third_party_policy NAME)
	set(Policy_${NAME} "Choose" CACHE STRING "How to get ${NAME}")
	set_property(CACHE Policy_${NAME} PROPERTY STRINGS "Choose" "Search on the path" "Download")
	if (Policy_${NAME} STREQUAL  "Download" OR Policy_ALL_3RD_PARTIES STREQUAL "Download all")
		find_package(${NAME} QUIET PATHS ${3RDPARTY_INSTALL_PREFIX} NO_DEFAULT_PATH )
		if (NOT ${NAME}_FOUND)
			include(cmake/DownloadAndInstall${NAME}.cmake)
			find_package(${NAME} REQUIRED PATHS ${3RDPARTY_INSTALL_PREFIX} NO_DEFAULT_PATH )
		endif()
	elseif (Policy_${NAME} STREQUAL  "Search on the path")
		find_package(${NAME} REQUIRED)
	else()
		message( SEND_ERROR "Policy for ${NAME} must be chosen!" )
	endif()
endmacro(third_party_policy)

### ZeroMQ
third_party_policy(ZeroMQ)
if(ZeroMQ_FOUND)
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
	set_target_properties(libzmq-static PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
endif()
### CPPZMQ
third_party_policy(CPPZMQ)
macro(requires_cppzmq NAME)
	target_link_libraries (${NAME} cppzmq-static)
	add_definitions(-DZMQ_STATIC)
endmacro(requires_cppzmq)
### FlatBuffers
third_party_policy(FlatBuffers)
if (FlatBuffers_FOUND)
	find_path(FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h PATHS "${3RDPARTY_INSTALL_PREFIX}/include")
endif()
macro(requires_flatbuffers)
	include_directories(${FLATBUFFERS_INCLUDE_DIR}) #targetinclude_directory????
endmacro(requires_flatbuffers)
### Eigen
third_party_policy(Eigen3)
macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

if (ADD_PLOTTER)
	### OpenCV
	set(OpenCV_STATIC TRUE)
	third_party_policy(OpenCV)
	if(OpenCV_FOUND)
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
		set_target_properties(IlmImf PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
	endif()
	macro(requires_opencv NAME)
		target_link_libraries ( ${NAME} opencv_core opencv_highgui opencv_imgcodecs opencv_imgproc ) # or ${OpenCV_LIBS}
	endmacro(requires_opencv)
endif(ADD_PLOTTER)

# Define the two required variables before including
# the source code for watching a git repository.
set(PRE_CONFIGURE_FILE "cmake/git.h.in")
set(POST_CONFIGURE_FILE "${CMAKE_BINARY_DIR}/git.h")
include(cmake/git_watcher.cmake)

#### Lib settings
set(LIB_VERSION_NAME "${SF_VERSION_MAJOR}_${SF_VERSION_MINOR}_${SF_VERSION_PATCH}")
macro(install_lib NAME HEADERS)
	add_dependencies( ${NAME} check_git_repository )
	set_target_properties(${NAME} PROPERTIES FOLDER "Lib"
			RELEASE_POSTFIX "${MSVC_TOOLSET}-mt-s-${LIB_VERSION_NAME}"
			RELWITHDEBINFO_POSTFIX "${MSVC_TOOLSET}-mt-swd-${LIB_VERSION_NAME}" #wd as with_debug
			MINSIZEREL_POSTFIX "${MSVC_TOOLSET}-mt-sms-${LIB_VERSION_NAME}" #ms as min_size
			DEBUG_POSTFIX "${MSVC_TOOLSET}-mt-sgd-${LIB_VERSION_NAME}"
			)
	target_include_directories(${NAME} INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)
		# $<INSTALL_INTERFACE:include/mylib>  # <prefix>/include/mylib TODO...
	install(TARGETS ${NAME} ARCHIVE DESTINATION lib)#/$<CONFIG>)
	install(FILES ${HEADERS} DESTINATION include/sf)
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


#TODO: package config!
# menteni, mely package-t keresi a intall_prefixen, melyeket a rendes pathon...
#TODO: verziók...
#TODO: test third parties after creating - collect them and this one...

#•todo: check cross compile...
#linux, ros compatibility?
#lib names: versions, configurations, generators...