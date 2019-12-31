# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	msg_generated.h
	msg2buf.h
	ZMQSubscriber.h
	ZMQPublisher.h
	)
	

set (SOURCES
	msg2buf.cpp
	ZMQSubscriber.cpp
	ZMQPublisher.cpp
	)
	
add_library (sf_communication STATIC ${SOURCES} ${HEADERS})

# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_core)

target_include_directories(sf_communication INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)

requires_cppzmq(sf_communication)
requires_flatbuffers()

set_target_properties(sf_communication PROPERTIES FOLDER "Lib")

install(TARGETS sf_communication LIBRARY DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/sf)