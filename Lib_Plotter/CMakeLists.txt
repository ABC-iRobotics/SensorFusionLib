# PROJECT(SensorFusion)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
		${Eigen_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/Lib_Common
		)

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
target_link_libraries ( Lib_Plotter ${OpenCV_LIBS} Eigen3::Eigen Lib_Common )

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")
