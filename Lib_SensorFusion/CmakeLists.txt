# PROJECT(SensorFusion)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (
		${Eigen_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/Lib_Common
		)

# Add all header and cpp files in the directory to the project
set (HEADERS
	BaseSystem.h
	Sensor.h
	System.h
	SystemManager.h
	PartialCholevski.h
	KalmanFilter.h
	WAUKF.h
	pinv.h
	)
	
set (SOURCES
	BaseSystem.cpp
	Sensor.cpp
	System.cpp
	SystemManager.cpp
	PartialCholevski.cpp
	KalmanFilter.cpp
	WAUKF.cpp
	pinv.cpp
	)
	

# Add the dinamic library target to the project
add_library (Lib_SensorFusion STATIC ${SOURCES} ${HEADERS})

# Link with dependency libraries
target_link_libraries (Lib_SensorFusion Eigen3::Eigen Lib_Common
		)
		
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MT")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} /MT")