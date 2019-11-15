# PROJECT(SensorFusion)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (
		${PROJECT_SOURCE_DIR}/Lib_SensorFusion
		${PROJECT_SOURCE_DIR}/Lib_GINOP
		${PROJECT_SOURCE_DIR}/Lib_Communication
		${PROJECT_SOURCE_DIR}/Lib_Common
		)

# Add all header and cpp files in the directory to the project
set (HEADERS
	)

set (SOURCES
	main.cpp
	)

# Add the dinamic library target to the project
add_executable (App_Filter ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (App_Filter Lib_SensorFusion Lib_GINOP Lib_Communication)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")