# PROJECT(SensorFusion)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (
		${Eigen_INCLUDE_DIRS}
		)

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
add_library (Lib_Common STATIC ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (Lib_Common Eigen3::Eigen)
		
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")