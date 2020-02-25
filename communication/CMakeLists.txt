# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	ClockSynchronizer.h
	Periphery.h
	Logger.h
	CentralUnit.h
	)
set (HEADERS_INL
	ZMQClockSynchronizer.h
	SPDLogging.h
	ZMQCommunication.h
	msgcontent2buf.h
	NetworkConfig.h
	)

set (SOURCES
	ZMQClockSynchronizer.cpp
	ClockSynchronizer.cpp
	SPDLogging.cpp
	ZMQCommunication.cpp
	msgcontent2buf.cpp
	Periphery.cpp
	Logger.cpp
	CentralUnit.cpp
	NetworkConfig.cpp
	)

set(root ${CMAKE_CURRENT_SOURCE_DIR})

source_group(h_inl FILES ${HEADERS_INL}  PREFIX "Resource Headers" TREE ${root})

add_library (sf_communication STATIC ${SOURCES} ${HEADERS} ${HEADERS_INL})


# Add the dinamic library target to the project
target_link_libraries (sf_communication sf_types)

requires_cppzmq(sf_communication)
requires_flatbuffers()
requires_spdlog(sf_communication)

install_lib(sf_communication "${HEADERS}")
