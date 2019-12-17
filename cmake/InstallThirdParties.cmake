file(MAKE_DIRECTORY ${CMAKE_3RDPARTY_DIR})
# CMake 3rd parties
message(STATUS "Getting generator and architecture...")
# Set the same generator
include(CheckSymbolExists) # ARM64???
if(WIN32)
	check_symbol_exists("_M_AMD64" "" RTC_ARCH_X64)
	if(NOT RTC_ARCH_X64)
	 check_symbol_exists("_M_IX86" "" RTC_ARCH_X86)
	 if(NOT RTC_ARCH_X86)
		set(RTC_ARCH_X64 TRUE)
	 endif()
	endif(NOT RTC_ARCH_X64)
else(WIN32)
	check_symbol_exists("__i386__" "" RTC_ARCH_X86)
	check_symbol_exists("__x86_64__" "" RTC_ARCH_X64)
	check_symbol_exists("__arm__" "" RTC_ARCH_ARM)
endif(WIN32)
if    (CMAKE_GENERATOR STREQUAL "Visual Studio 16 2019")
	if(RTC_ARCH_X86)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} -AWin32") #TODO test it...
	elseif(RTC_ARCH_X64)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} -AWin64") #TODO test it...
	elseif(RTC_ARCH_ARM)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} -AARM") #TODO test it...
	endif()
elseif ((CMAKE_GENERATOR STREQUAL "Visual Studio 15 2017") OR
		(CMAKE_GENERATOR STREQUAL "Visual Studio 14 2015") OR
		(CMAKE_GENERATOR STREQUAL "Visual Studio 12 2013") OR
		(CMAKE_GENERATOR STREQUAL "Visual Studio 11 2012"))
	if(RTC_ARCH_X86)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR}")
	elseif(RTC_ARCH_X64)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} Win64")
	elseif(RTC_ARCH_ARM)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} ARM")
	endif()
elseif((CMAKE_GENERATOR STREQUAL "Visual Studio 10 2010") OR (CMAKE_GENERATOR STREQUAL "Visual Studio 9 2008"))
	if(RTC_ARCH_X86)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR}") #TODO test it...
	elseif(RTC_ARCH_X64)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} Win64")
	elseif(RTC_ARCH_ARM)
		set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR} IA64")
	endif()
else()
	set( GENERATOR_ARGUMENT "-G${CMAKE_GENERATOR}")
endif()
message (STATUS "  ...succeded: ${GENERATOR_ARGUMENT}")

# Call configure, generate and build (Release & Debug) on the externals
message(STATUS "Generating solution to install 3rd parties")
EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_SOURCE_DIR}/cmake\" ARGS 
	\"${GENERATOR_ARGUMENT}\" \"-S${CMAKE_SOURCE_DIR}/cmake\" \"-B${CMAKE_3RDPARTY_DIR}\" -DBUILD_SUBTREE=1 )
message(STATUS "Downloading and building third parties (RELEASE). It can take some time...")
EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_SOURCE_DIR}/cmake\" ARGS --build \"${CMAKE_3RDPARTY_DIR}\" --config Release )	
if (WIN32)
	message(STATUS "Building third parties (DEBUG). It can take some time...")
	EXEC_PROGRAM(${CMAKE_COMMAND} \"${CMAKE_SOURCE_DIR}/cmake\" ARGS --build \"${CMAKE_3RDPARTY_DIR}\" --config Debug )
endif()
message(STATUS "3rd party building phase done!")


find_package (Eigen3 3.3 REQUIRED NO_MODULE PATHS ${CMAKE_3RDPARTY_DIR}/src/Ext-Eigen3-build NO_DEFAULT_PATH)
set( ZMQ_SRC_DIR			${CMAKE_3RDPARTY_DIR}/src/Ext-zmqlib )
set( ZMQ_BUILD_DIR			${CMAKE_3RDPARTY_DIR}/src/Ext-zmqlib-build )
find_package(ZeroMQ PATHS ${ZMQ_BUILD_DIR} NO_DEFAULT_PATH)
find_package(cppzmq REQUIRED PATHS ${CMAKE_3RDPARTY_DIR}/src/Ext-cppzmq-build NO_DEFAULT_PATH)
set( ZMQLIB_INCLUDE_DIRS	${ZMQ_SRC_DIR}/ ${ZMQ_SRC_DIR}/include)
set( CPPZMQ_INCLUDE_DIR		${CMAKE_3RDPARTY_DIR}/src/Ext-cppzmq ${ZMQLIB_INCLUDE_DIRS})
find_package(OpenCV REQUIRED PATHS ${CMAKE_3RDPARTY_DIR}/src/Ext-opencv-build NO_DEFAULT_PATH)

macro(requires_eigen NAME)
	include_directories ( ${Eigen_INCLUDE_DIRS}	)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

macro(requires_cppzmq NAME)
	include_directories ( ${CPPZMQ_INCLUDE_DIR}	)
	target_link_libraries (${NAME} cppzmq-static)
	add_definitions(-DZMQ_STATIC)
endmacro(requires_cppzmq)

macro(requires_flatbuffers NAME)
	include_directories ( ${CMAKE_3RDPARTY_DIR}/src/Ext-flatbuffers/include )
endmacro(requires_flatbuffers)

macro(requires_opencv NAME)
	include_directories(${OpenCV_INCLUDE_DIRS})
	target_link_libraries ( ${NAME} ${OpenCV_LIBS} )
endmacro(requires_opencv)