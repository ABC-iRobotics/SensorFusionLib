# PROJECT(SensorFusion)

include_directories (
		${PROJECT_SOURCE_DIR}/Lib_Common
		)

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
	
add_library (Lib_Communication STATIC ${SOURCES} ${HEADERS})

# Add the dinamic library target to the project
target_link_libraries (Lib_Communication Lib_Common)

requires_cppzmq(Lib_Communication)
requires_flatbuffers(Lib_Communication)