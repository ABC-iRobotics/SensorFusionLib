# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	IClockSynchronizer.h
	Periphery.h
	)
set (HEADERS_INL
	ILogReader.h
	SenderToSPDLog.h
	spdLogRead.h
	ZMQClockSynchronizer.h
	ZMQCommunication.h
	msgcontent2buf.h
	)

set (SOURCES
	SenderToSPDLog.cpp
	spdLogRead.cpp
	ZMQClockSynchronizer.cpp
	IClockSynchronizer.cpp
	ZMQCommunication.cpp
	msgcontent2buf.cpp
	Periphery.cpp
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
