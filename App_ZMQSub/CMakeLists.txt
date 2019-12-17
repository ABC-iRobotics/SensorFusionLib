# PROJECT(SensorFusion)

include_directories (
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
add_executable (App_ZMQSub ${SOURCES} ${HEADERS} )

# Link with dependency libraries
target_link_libraries (App_ZMQSub Lib_Communication )