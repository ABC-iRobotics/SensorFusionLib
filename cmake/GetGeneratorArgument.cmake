function(GetGeneratorArgument RES)
  #message(STATUS "Getting generator and architecture...")
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
			set( ${RES} "-G${CMAKE_GENERATOR} -AWin32" PARENT_SCOPE) #TODO test it...
		elseif(RTC_ARCH_X64)
			set( ${RES} "-G${CMAKE_GENERATOR} -AWin64" PARENT_SCOPE) #TODO test it...
		elseif(RTC_ARCH_ARM)
			set( ${RES} "-G${CMAKE_GENERATOR} -AARM" PARENT_SCOPE) #TODO test it...
		endif()
	elseif ((CMAKE_GENERATOR STREQUAL "Visual Studio 15 2017") OR
			(CMAKE_GENERATOR STREQUAL "Visual Studio 14 2015") OR
			(CMAKE_GENERATOR STREQUAL "Visual Studio 12 2013") OR
			(CMAKE_GENERATOR STREQUAL "Visual Studio 11 2012"))
		if(RTC_ARCH_X86)
			set( ${RES} "-G${CMAKE_GENERATOR}" PARENT_SCOPE)
		elseif(RTC_ARCH_X64)
			set( ${RES} "-G${CMAKE_GENERATOR} Win64" PARENT_SCOPE)
		elseif(RTC_ARCH_ARM)
			set( ${RES} "-G${CMAKE_GENERATOR} ARM" PARENT_SCOPE)
		endif()
	elseif((CMAKE_GENERATOR STREQUAL "Visual Studio 10 2010") OR (CMAKE_GENERATOR STREQUAL "Visual Studio 9 2008"))
		if(RTC_ARCH_X86)
			set( ${RES} "-G${CMAKE_GENERATOR}" PARENT_SCOPE) #TODO test it...
		elseif(RTC_ARCH_X64)
			set( ${RES} "-G${CMAKE_GENERATOR} Win64" PARENT_SCOPE)
		elseif(RTC_ARCH_ARM)
			set( ${RES} "-G${CMAKE_GENERATOR} IA64" PARENT_SCOPE)
		endif()
	else()
		set( ${RES} "-G${CMAKE_GENERATOR}" PARENT_SCOPE)
	endif()
	#message (STATUS "  ...succeded: ${${RES}}")
endfunction()