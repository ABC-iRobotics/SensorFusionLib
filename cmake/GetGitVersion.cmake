# The script defines
# GIT_RETRIEVED_STATE : true/false according to the success
# GIT_VERSION_MAJOR
# GIT_VERSION_MINOR
# GIT_VERSION_PATCH
# GIT_VERSION_COMMIT
# GIT_TAG_VERSION = GIT_VERSION_MAJOR.GIT_VERSION_MINOR.GIT_VERSION_PATCH
# GIT_VERSION = GIT_VERSION_MAJOR.GIT_VERSION_MINOR.GIT_VERSION_PATCH-GIT_VERSION_COMMIT
# GIT_IS_DIRTY = true/false
# GIT_VERSION_FULL = GIT_VERSION / GIT_VERSION-dirty
#
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
	
# Create GIT_VERSION_FULL
string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" GIT_VERSION_MAJOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.([0-9]+)\\..*" "\\1" GIT_VERSION_MINOR "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)-?.*" "\\1" GIT_VERSION_PATCH "${SF_VERSION}")
string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+-?([a-zA-Z0-9]*-?[a-zA-Z0-9]*)$" "\\1" GIT_VERSION_COMMIT "${SF_VERSION}")

set(GIT_VERSION_FULL ${SF_VERSION})
if (GIT_IS_DIRTY)
	set(GIT_VERSION_FULL "${GIT_VERSION_FULL}-dirty")
endif()

set(export_config_name "SensorFusion")

set(GIT_TAG_VERSION "${GIT_VERSION_MAJOR}.${GIT_VERSION_MINOR}.${GIT_VERSION_PATCH}")