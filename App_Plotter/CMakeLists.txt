# PROJECT(SensorFusion)

include_directories (
		${PROJECT_SOURCE_DIR}/Lib_Communication
		${PROJECT_SOURCE_DIR}/Lib_Plotter
		${PROJECT_SOURCE_DIR}/Lib_Common
		)

# Add all header and cpp files in the directory to the project
set (HEADERS	)

set (SOURCES
	main.cpp
	)

# Add the dinamic library target to the project
add_executable (App_Plotter ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (App_Plotter Lib_Communication Lib_Plotter)

requires_eigen(App_Plotter)
requires_zmq(App_Plotter)
set_target_properties(App_Plotter PROPERTIES FOLDER "SensorFusion/App")