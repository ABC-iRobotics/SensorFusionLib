# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	ClockSynchronizer.h
	Periphery.h
	ZMQClockSynchronizer.h
	msgcontent2buf.h
	NetworkConfig.h
	Forwarder.h
	SteppableSimulator.h
	RealTimeSimulator.h
	ZMQReciever.h
	comm_defs.h
	Filter.h
	Logger.h
	SPDLogReader.h
	msg_old2buf.h
	msg_old_generated.h
	)

set (SOURCES
	ZMQClockSynchronizer.cpp
	ClockSynchronizer.cpp
	msgcontent2buf.cpp
	Periphery.cpp
	NetworkConfig.cpp
	Forwarder.cpp
	SteppableSimulator.cpp
	RealTimeSimulator.cpp
	ZMQReciever.cpp
	Filter.cpp
	Logger.cpp
	SPDLogReader.cpp
	msg_old2buf.cpp
	)

source_group("Background headers" FILES
	msgcontent2buf.h
	comm_defs.h
	SPDLogReader.h
	ZMQClockSynchronizer.h
	ClockSynchronizer.h
	NetworkConfig.h
	ZMQReciever.h
	msg_old2buf.h
	msg_old_generated.h
	)

add_library (sf_communication STATIC ${SOURCES} ${HEADERS})


# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_types)

requires_cppzmq(sf_communication)
requires_flatbuffers()
requires_spdlog(sf_communication)

install_lib(sf_communication "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})
