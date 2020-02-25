add_subdirectory(common)

# ZMQ tests
if (ZeroMQ_FOUND)
	option(BUILD_ZMQ_TESTS "To include zmq tests" FALSE)
	if(BUILD_ZMQ_TESTS)
		add_subdirectory(zmq)
	endif()
endif()

# SF tests
set(tests
	test_spd_log
	test_clocksync_local
	test_zmqsub_zmqpub
	test_communication
	test_configfile_init
	)
foreach(test ${tests})
	add_executable(${test} ${test}.cpp)
	set_target_properties(${test} PROPERTIES FOLDER "Tests")
	target_link_libraries(${test} unity)

	if(WIN32)
		add_test(NAME ${test} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH} COMMAND ${test})
	else()
		add_test(NAME ${test} COMMAND ${test})
	endif()
	set_tests_properties(${test} PROPERTIES TIMEOUT 20)
	set_tests_properties(${test} PROPERTIES SKIP_RETURN_CODE 77)
endforeach()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/networkconfig_local.json
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(test_spd_log sf_communication)
target_link_libraries(test_clocksync_local sf_communication)
target_link_libraries(test_zmqsub_zmqpub sf_communication)
target_link_libraries(test_communication sf_communication)
target_link_libraries(test_configfile_init sf_communication)
