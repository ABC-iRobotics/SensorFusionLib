# PROJECT(SensorFusion)

# Add all header and cpp files in the directory to the project
set (HEADERS	)

set (SOURCES
	main.cpp
	)

# Add the dinamic library target to the project
add_executable (PlotterTest ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (PlotterTest sf_communication sf_plotter)

set_target_properties(PlotterTest PROPERTIES FOLDER "Tests")