# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	BaseSystem.h
	Sensor.h
	System.h
	SystemManager.h
	#PartialCholevski.h
	KalmanFilter.h
	WAUKF.h
	pinv.h )
	
set (SOURCES
	BaseSystem.cpp
	Sensor.cpp
	System.cpp
	SystemManager.cpp
	#PartialCholevski.cpp
	KalmanFilter.cpp
	WAUKF.cpp
	pinv.cpp )

# Add the dinamic library target to the project
add_library (sf_core STATIC ${SOURCES} ${HEADERS} )

# Link with dependency libraries
target_link_libraries (sf_core sf_types relaxedUT)

install_lib(sf_core "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})