# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	StatisticValue.h
	defs.h
	DataMsg.h
	PrintNestedException.h
	FilterCore.h
	)

set (SOURCES
	StatisticValue.cpp
	defs.cpp
	DataMsg.cpp
	PrintNestedException.cpp
	)

# Add the dinamic library target to the project
add_library (sf_types STATIC ${SOURCES} ${HEADERS})

requires_eigen(sf_types)

install_lib(sf_types "${HEADERS}")