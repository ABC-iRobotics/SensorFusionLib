# PROJECT(SensorFusion)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#file(COPY ${OpenCV_BINARY_DIRS}/3rdParty DESTINATION ${CMAKE_BINARY_DIR}/3rdParty)

#include_directories (${CMAKE_BINARY_DIR}/dependencies/assimp/include
#			${CMAKE_BINARY_DIR}/dependencies/v-hacd/src/VHACD_Lib/public#
#			${CMAKE_BINARY_DIR}/dependencies/v-hacd/src/VHACD_Lib/inc
#			${CMAKE_BINARY_DIR}/dependencies/MeshDecimation/src		
#			${CMAKE_BINARY_DIR}/dependencies/qhull-2012.1/src
#			${CMAKE_BINARY_DIR}/dependencies/qhull-2012.1/src/libqhullp
#			${CMAKE_BINARY_DIR}/dependencies/qhull-2012.1/src/libqhullcpp
#			)

# Add all header and cpp files in the directory to the project
set (HEADERS
	color.h
	cvplot.h
	figure.h
	highgui.h
	internal.h
	window.h
	plotter.h
	)
	
set (SOURCES
	color.cc
	figure.cc
	highgui.cc
	window.cc
	plotter.cpp
	)



# Add the dinamic library target to the project
add_library (PlotterLib STATIC ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries ( PlotterLib ${OpenCV_LIBS}
		)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")
