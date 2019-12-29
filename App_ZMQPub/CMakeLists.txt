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
add_executable (App_ZMQPub ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (App_ZMQPub Lib_Communication)
requires_eigen(App_ZMQPub)
requires_zmq(App_ZMQPub)
set_target_properties(App_ZMQPub PROPERTIES FOLDER "SensorFusion/App")