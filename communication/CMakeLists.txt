# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	)
set (HEADERS_INL
	msg_generated.h
	ILogReader.h
	msg2buf.h
	spdLogWrite.h
	SPDLogRead.h
	ZMQClockSynchronizer.h
	)

set (SOURCES
	msg2buf.cpp
	spdLogWrite.cpp
	SPDLogRead.cpp
	ZMQClockSynchronizer.cpp
	)

add_library (sf_communication STATIC ${SOURCES} ${HEADERS})

# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_types)

requires_cppzmq(sf_communication)
requires_flatbuffers()
requires_spdlog(sf_communication)

install_lib(sf_communication "${HEADERS}")