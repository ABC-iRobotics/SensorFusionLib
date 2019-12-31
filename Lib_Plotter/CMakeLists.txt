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
add_library (Lib_Plotter STATIC ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries ( Lib_Plotter Lib_Common )

target_include_directories(Lib_Plotter INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>)

requires_opencv(Lib_Plotter)

set_target_properties(Lib_Plotter PROPERTIES FOLDER "Lib")

install(TARGETS Lib_Plotter LIBRARY DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include/sf)