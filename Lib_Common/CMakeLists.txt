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
	

#include_directories ( "${CMAKE_3RDPARTY_DIR}/Ext-Eigen3" )
requires_eigen(Lib_Common)


# Add the dinamic library target to the project
add_library (Lib_Common STATIC ${SOURCES} ${HEADERS})

set_target_properties(Lib_Common PROPERTIES FOLDER "SensorFusion/Lib")
