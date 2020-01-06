# Check the optional git variable.
# If it's not set, we'll try to find it using the CMake packaging system.
if(NOT DEFINED GIT_EXECUTABLE)
    find_package(Git QUIET REQUIRED)
endif()

# Get the hash for HEAD.
set(GIT_RETRIEVED_STATE "true")
execute_process(COMMAND
    "${GIT_EXECUTABLE}" rev-parse --verify HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE GIT_HEAD_SHA1
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT res EQUAL 0)
    set(GIT_RETRIEVED_STATE "false")
    set(GIT_HEAD_SHA1 "GIT-NOTFOUND")
endif()

# Get whether or not the working tree is dirty.
execute_process(COMMAND
    "${GIT_EXECUTABLE}" status --porcelain
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE out
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT res EQUAL 0)
    set(GIT_RETRIEVED_STATE "false")
    set(GIT_IS_DIRTY "false")
else()
    if(NOT "${out}" STREQUAL "")
        set(GIT_IS_DIRTY "true")
    else()
        set(GIT_IS_DIRTY "false")
    endif()
endif()

# Get the version
execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags
	WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
	RESULT_VARIABLE res
	OUTPUT_VARIABLE SF_VERSION
	ERROR_QUIET
	OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT res EQUAL 0)
    set(GIT_RETRIEVED_STATE "false")
    set(GIT_IS_DIRTY "false")
	set(SF_VERSION "NOTFOUND")
endif()
	
# Create SF_VERSION_FULL
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" GIT_VERSION_MAJOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+)\\..*" "\\1" GIT_VERSION_MINOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)-?.*" "\\1" GIT_VERSION_PATCH "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+-?([a-zA-Z0-9]*-?[a-zA-Z0-9]*)$" "\\1" GIT_VERSION_COMMIT "${SF_VERSION}")

set(SF_VERSION_FULL ${SF_VERSION})
if (GIT_IS_DIRTY)
	set(SF_VERSION_FULL "${SF_VERSION_FULL}-dirty")
endif()

set(export_config_name "SensorFusion")

set(SF_PACKAGE_VERSION "${GIT_VERSION_MAJOR}.${GIT_VERSION_MINOR}.${GIT_VERSION_PATCH}")
# Create versioning/ConfigVersionCheck.cmake
file(WRITE "${CMAKE_INSTALL_PREFIX}/cmake/${export_config_name}Version.cmake"
"set(SF_VERSION_FULL \"${SF_VERSION_FULL}\")
set(SF_VERSION \"${SF_PACKAGE_VERSION}\")")

#----------
# Write verisoning
message(STATUS "${CMAKE_INSTALL_PREFIX}/cmake/${export_config_name}VersionCheck-${BUILD_TYPE}.cmake")
file(WRITE "${CMAKE_INSTALL_PREFIX}/cmake/${export_config_name}VersionCheck-${BUILD_TYPE}.cmake"
"if(NOT SF_VERSION_FULL STREQUAL \"${SF_VERSION_FULL}\")
	message(FATAL_ERROR \"${export_config_name} package: wrong ${BUILD_TYPE} version\")
endif()")

#------------------------------------------------------------------------------
# Configure <export_config_name>ConfigVersion.cmake common to build and install tree
message(STATUS "${SF_PACKAGE_VERSION}")
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_INSTALL_PREFIX}/${export_config_name}ConfigVersion.cmake
	VERSION "${SF_PACKAGE_VERSION}"
	COMPATIBILITY ExactVersion)

configure_file("${CMAKE_CURRENT_LIST_DIR}/sf_version.h.in"
	"${CMAKE_INSTALL_PREFIX}/include/sf_version.h" @ONLY)

configure_file("${CMAKE_CURRENT_LIST_DIR}/SensorFusionConfig.cmake.in"
	"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake" @ONLY)