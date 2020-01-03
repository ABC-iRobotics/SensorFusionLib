# git_watcher.cmake
#
# License: MIT
# Based on: https://raw.githubusercontent.com/andrew-hardin/cmake-git-version-tracking/master/git_watcher.cmake

macro(PATH_TO_ABSOLUTE var_name)
    get_filename_component(${var_name} "${${var_name}}" ABSOLUTE)
endmacro()

# Check that a required variable is set.
macro(CHECK_REQUIRED_VARIABLE var_name)
    if(NOT DEFINED ${var_name})
        message(FATAL_ERROR "The \"${var_name}\" variable must be defined.")
    endif()
    PATH_TO_ABSOLUTE(${var_name})
endmacro()

# Check that an optional variable is set, or, set it to a default value.
macro(CHECK_OPTIONAL_VARIABLE var_name default_value)
    if(NOT DEFINED ${var_name})
        set(${var_name} ${default_value})
    endif()
    PATH_TO_ABSOLUTE(${var_name})
endmacro()

CHECK_OPTIONAL_VARIABLE(GIT_STATE_FILE "${CMAKE_BINARY_DIR}/versioning/git-state")
CHECK_OPTIONAL_VARIABLE(GIT_WORKING_DIR "${CMAKE_SOURCE_DIR}")

# Check the optional git variable.
# If it's not set, we'll try to find it using the CMake packaging system.
if(NOT DEFINED GIT_EXECUTABLE)
    find_package(Git QUIET REQUIRED)
endif()
CHECK_REQUIRED_VARIABLE(GIT_EXECUTABLE)






# Function: GetGitState
# Description: gets the current state of the git repo.
# Args:
#   _working_dir (in)  string; the directory from which git commands will be executed.
#   _state       (out) list; a collection of variables representing the state of the
#                            repository (e.g. commit SHA).
function(GetGitState _working_dir _state)

    # Get the hash for HEAD.
    set(_success "true")
    execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --verify HEAD
        WORKING_DIRECTORY "${_working_dir}"
        RESULT_VARIABLE res
        OUTPUT_VARIABLE _hashvar
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT res EQUAL 0)
        set(_success "false")
        set(_hashvar "GIT-NOTFOUND")
    endif()

    # Get whether or not the working tree is dirty.
    execute_process(COMMAND
        "${GIT_EXECUTABLE}" status --porcelain
        WORKING_DIRECTORY "${_working_dir}"
        RESULT_VARIABLE res
        OUTPUT_VARIABLE out
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT res EQUAL 0)
        set(_success "false")
        set(_dirty "false")
    else()
        if(NOT "${out}" STREQUAL "")
            set(_dirty "true")
        else()
            set(_dirty "false")
        endif()
    endif()

	# Get the version
	execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags
		WORKING_DIRECTORY ${_working_dir}
		RESULT_VARIABLE res
		OUTPUT_VARIABLE _version
		ERROR_QUIET
		OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT res EQUAL 0)
        set(_success "false")
        set(_dirty "false")
		set(_version "NOTFOUND")
    endif()
    # Return a list of our variables to the parent scope.
    set(${_state} ${_success} ${_hashvar} ${_dirty} ${_version} PARENT_SCOPE)
endfunction()

if(NOT DEFINED _BUILD_TIME_CHECK_GIT)
	# Scope of the original cmake script at compiler stage:
	# this function sets up custom commands that make the build system
	# check the state of git before every build. If the state has
	# changed, then a file is configured.
	set(PRE_CONFIGURE_FILE "${CMAKE_SOURCE_DIR}/cmake/sf_version.h.in")
	set(POST_CONFIGURE_FILE "${CMAKE_BINARY_DIR}/versioning/sf_version.h")
	add_custom_target(check_git_repository
	ALL
	DEPENDS ${PRE_CONFIGURE_FILE}
	BYPRODUCTS ${POST_CONFIGURE_FILE}
	COMMENT "Checking the git repository for changes..."
	COMMAND
		${CMAKE_COMMAND}
		-D_BUILD_TIME_CHECK_GIT=TRUE
		-DGIT_WORKING_DIR=${GIT_WORKING_DIR}
		-DGIT_EXECUTABLE=${GIT_EXECUTABLE}
		-DGIT_STATE_FILE=${GIT_STATE_FILE}
		-DPRE_CONFIGURE_FILE=${PRE_CONFIGURE_FILE}
		-DPOST_CONFIGURE_FILE=${POST_CONFIGURE_FILE}
		-DORIGINAL_SOURCE_FOLDER=${CMAKE_SOURCE_DIR}
		-P "${CMAKE_CURRENT_LIST_FILE}")
else()
	# Check if the repo has changed. If so, run the change action.
    # Get the current state of the repo.
    GetGitState("${GIT_WORKING_DIR}" state)

    # Check if the state has changed compared to the backup on disk.
    if(EXISTS "${GIT_STATE_FILE}")
        file(READ "${GIT_STATE_FILE}" OLD_HEAD_CONTENTS)
        if(OLD_HEAD_CONTENTS STREQUAL "${state}")
            # State didn't change.
            return()
        endif()
    endif()

    # The state has changed.
    # We need to update the state file on disk.
    # Future builds will compare their state to this file.
    file(WRITE "${GIT_STATE_FILE}" "${state}")
	
	# Create SF_VERSION_FULL
	LIST(GET state 3 SF_VERSION_FULL)
	string(REGEX REPLACE "^([0-9]+)\\..*" "\\1" GIT_VERSION_MAJOR "${SF_VERSION_FULL}")
	string(REGEX REPLACE "^[0-9]+\\.([0-9]+)\\..*" "\\1" GIT_VERSION_MINOR "${SF_VERSION_FULL}")
	string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.([0-9]+)-?.*" "\\1" GIT_VERSION_PATCH "${SF_VERSION_FULL}")
	string(REGEX REPLACE "^[0-9]+\\.[0-9]+\\.[0-9]+-?([a-zA-Z0-9]*-?[a-zA-Z0-9]*)$" "\\1" GIT_VERSION_COMMIT "${SF_VERSION_FULL}")
	LIST(GET state 2 GIT_IS_DIRTY)
	if (GIT_IS_DIRTY)
		set(SF_VERSION_FULL "${SF_VERSION_FULL}-dirty")
	endif()
	
	# Create versioning/ConfigVersionCheck.cmake
	configure_file("${ORIGINAL_SOURCE_FOLDER}/cmake/ConfigVersionCheck.cmake.in" "${CMAKE_BINARY_DIR}/versioning/ConfigVersionCheck.cmake" @ONLY)
	configure_file("${ORIGINAL_SOURCE_FOLDER}/cmake/SensorFusionConfigVersion.cmake.in"
					"${CMAKE_BINARY_DIR}/versioning/SensorFusionConfigVersion.cmake" @ONLY)
					
	# Create versioning/ConfigVersionCheck.cmake
	file(WRITE "${CMAKE_BINARY_DIR}/versioning/Version.cmake" "set(SF_VERSION_FULL \"${SF_VERSION_FULL}\")")

	# Set variables by index, then configure the file w/ these variables defined.
	LIST(GET state 0 GIT_RETRIEVED_STATE)
	LIST(GET state 1 GIT_HEAD_SHA1)
	LIST(GET state 2 GIT_IS_DIRTY)
	configure_file("${PRE_CONFIGURE_FILE}" "${POST_CONFIGURE_FILE}" @ONLY)
endif()

