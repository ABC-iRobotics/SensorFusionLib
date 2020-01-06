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

find_package(SensorFusion 0.0.1)

add_executable(sg "main.cpp")

target_link_libraries(sg SensorFusion::sf_core SensorFusion::sf_communication SensorFusion::sf_plotter)