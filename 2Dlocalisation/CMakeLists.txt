
# Add all header and cpp files in the directory to the project
set (HEADERS
	Sensor2DPose.h
	Sensor2DPosewCalibration.h
	Sensor2DPosewDrift.h
	Vechicle2D.h )
	
set (SOURCES
	Sensor2DPose.cpp
	Sensor2DPosewCalibration.cpp
	Sensor2DPosewDrift.cpp
	Vechicle2D.cpp )

# Add the dinamic library target to the project
add_library (2Dlocalisation STATIC ${SOURCES} ${HEADERS} )

# Link with dependency libraries
target_link_libraries (2Dlocalisation sf_core )

install_lib(2Dlocalisation "${HEADERS}")