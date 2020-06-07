# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	ClockSynchronizer.h
	Periphery.h
	ZMQClockSynchronizer.h
	msgcontent2buf.h
	NetworkConfig.h
	Forwarder.h
	ZMQReciever.h
	comm_defs.h
	Logger.h
	SPDLogReader.h
	)

set (SOURCES
	ZMQClockSynchronizer.cpp
	ClockSynchronizer.cpp
	msgcontent2buf.cpp
	Periphery.cpp
	NetworkConfig.cpp
	Forwarder.cpp
	ZMQReciever.cpp
	Logger.cpp
	SPDLogReader.cpp
	)

add_library (sf_communication STATIC ${SOURCES} ${HEADERS})


# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_types)

requires_cppzmq(sf_communication)
requires_flatbuffers()
requires_spdlog(sf_communication)

install_lib(sf_communication "${HEADERS}")
