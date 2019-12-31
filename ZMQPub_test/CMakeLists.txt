# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	)
	

set (SOURCES
	main.cpp
	)
	

# Add the dinamic library target to the project
add_executable (ZMQPubTEST ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (ZMQPubTEST sf_communication)

set_target_properties(ZMQPubTEST PROPERTIES FOLDER "Tests")