# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	msg_generated.h
	msg2buf.h
	ZMQSubscriber.h
	ZMQPublisher.h
	spdLogWrite.h
	spdLogRead.h
	ClockSynchronizer.h
	)

set (SOURCES
	msg2buf.cpp
	ZMQSubscriber.cpp
	ZMQPublisher.cpp
	spdLogWrite.cpp
	spdLogRead.cpp
	ClockSynchronizer.cpp
	)

add_library (sf_communication STATIC ${SOURCES} ${HEADERS})

# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_types)

requires_cppzmq(sf_communication)
requires_flatbuffers()
requires_spdlog(sf_communication)

install_lib(sf_communication "${HEADERS}")