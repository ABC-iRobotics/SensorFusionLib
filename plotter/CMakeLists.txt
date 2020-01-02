# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS
	color.h
	cvplot.h
	figure.h
	highgui.h
	internal.h
	window.h
	plotter.h
	FilterPlot.h
	)
	
set (SOURCES
	color.cc
	figure.cc
	highgui.cc
	window.cc
	plotter.cpp
	FilterPlot.cpp
	)

# Add the dinamic library target to the project
add_library (sf_plotter STATIC ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries ( sf_plotter sf_types )

requires_opencv(sf_plotter)

install_lib(sf_plotter "${HEADERS}")