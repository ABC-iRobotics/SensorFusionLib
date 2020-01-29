add_subdirectory(common)

# ZMQ tests
if (ZeroMQ_FOUND)
	option(BUILD_ZMQ_TESTS "To include zmq tests" FALSE)
	if(BUILD_ZMQ_TESTS)
		add_subdirectory(zmq)
	endif()
endif()