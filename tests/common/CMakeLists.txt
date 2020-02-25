# Add unity target
add_library(unity STATIC
	"${CMAKE_CURRENT_LIST_DIR}/unity.c"
	"${CMAKE_CURRENT_LIST_DIR}/unity.h"
	"${CMAKE_CURRENT_LIST_DIR}/unity_internals.h")
set_target_properties(unity PROPERTIES
	PUBLIC_HEADER "${CMAKE_CURRENT_LIST_DIR}/unity.h"
	FOLDER "Tests")
target_compile_definitions(unity PUBLIC "UNITY_USE_COMMAND_LINE_ARGS" "UNITY_EXCLUDE_FLOAT")
target_include_directories(unity PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

