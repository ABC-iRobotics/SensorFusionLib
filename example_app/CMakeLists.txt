project(TesterApp)
CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

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

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Bin) #Or the program puts the application into different folders...

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/networkconfig_1.json
	DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt
	DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

find_package(SensorFusion 0.0.1)

add_subdirectory (periphery)
add_subdirectory (periphery_w_clocksync)
add_subdirectory (logger)
