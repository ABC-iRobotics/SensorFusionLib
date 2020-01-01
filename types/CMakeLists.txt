# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	StatisticValue.h
	defs.h
	DataMsg.h
	TimeMicroSec.h
	)
	
set (SOURCES
	StatisticValue.cpp
	DataMsg.cpp
	TimeMicroSec.cpp
	)
	

# Add the dinamic library target to the project
add_library (sf_types STATIC ${SOURCES} ${HEADERS})

set_target_properties(sf_types PROPERTIES FOLDER "Lib")

target_include_directories(sf_types INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)
	# $<INSTALL_INTERFACE:include/mylib>  # <prefix>/include/mylib TODO...
requires_eigen(sf_types)

install_lib(sf_types "${HEADERS}")