# Add testutil-static target
set(TESTUTIL_SOURCES
	testutil.cpp
	testutil.hpp
	testutil_monitoring.cpp
	testutil_monitoring.hpp
	testutil_security.cpp
	testutil_security.hpp
	testutil_unity.cpp
	testutil_unity.hpp
	)
add_library(testutil-static STATIC ${TESTUTIL_SOURCES})
set_target_properties(testutil-static PROPERTIES FOLDER "Tests/ZMQ")
target_link_libraries(testutil-static libzmq-static ${OPTIONAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} unity)

# use testutil-static for both tests and unit tests
set(TESTUTIL_LIB testutil-static)
if(MSVC_VERSION LESS 1700)
	set_source_files_properties("${CMAKE_CURRENT_LIST_DIR}/../common/unity.c" PROPERTIES LANGUAGE CXX)
endif()
if(MSVC_VERSION LESS 1600)
	target_compile_definitions(unity PUBLIC "UNITY_EXCLUDE_STDINT_H")
endif()
# add include dirs for all targets
#if (0)
#	include_directories("${ZeroMQ_SOURCE_DIR}/../include" "${ZeroMQ_BINARY_DIR}")
#	if(WIN32)
#	  add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
#	endif()
#endif()


if(WIN32)
	add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
endif()

set(tests
	test_ancillaries
	test_system
	test_pair_inproc
	test_pair_tcp
	test_reqrep_inproc
	test_reqrep_tcp
	test_hwm
	test_hwm_pubsub
	test_reqrep_device
	test_sub_forward
	test_invalid_rep
	test_msg_flags
	test_msg_ffn
	test_connect_resolve
	test_immediate
	test_last_endpoint
	test_term_endpoint
	test_router_mandatory
	test_probe_router
	test_stream
	test_stream_empty
	test_stream_disconnect
	test_disconnect_inproc
	test_unbind_wildcard
	test_ctx_options
	test_ctx_destroy
	test_security_no_zap_handler
	test_security_null
	test_security_plain
	test_security_zap
	test_iov
	test_spec_req
	test_spec_rep
	test_spec_dealer
	test_spec_router
	test_spec_pushpull
	test_req_correlate
	test_req_relaxed
	test_conflate
	test_inproc_connect
	test_issue_566
	test_shutdown_stress
	test_timeo
	test_many_sockets
	test_diffserv
	test_connect_rid
	test_xpub_nodrop
	test_pub_invert_matching
	test_setsockopt
	test_sockopt_hwm
	test_heartbeats
	test_atomics
	test_bind_src_address
	test_capabilities
	test_metadata
	test_router_handover
	test_srcfd
	test_stream_timeout
	test_xpub_manual
	test_xpub_welcome_msg
	test_xpub_verbose
	test_base85
	test_bind_after_connect_tcp
	test_sodium
	test_monitor
	test_socket_null
	test_reconnect_ivl
	test_mock_pub_sub
)

if(NOT WIN32)
  list(APPEND tests test_security_gssapi)
  list(APPEND tests
    test_ipc_wildcard
    test_pair_ipc
    test_rebind_ipc
    test_reqrep_ipc
    test_proxy
    test_proxy_hwm
    test_proxy_single_socket
    test_proxy_terminate
    test_getsockopt_memset
    test_filter_ipc
    test_stream_exceeds_buffer
    test_router_mandatory_hwm
    test_use_fd
    test_zmq_poll_fd
  )
  if(HAVE_FORK)
    list(APPEND tests test_fork)
  endif()
  if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    list(APPEND tests
      test_abstract_ipc
      )
    if(ZMQ_HAVE_TIPC)
      list(APPEND tests
        test_address_tipc
        test_pair_tipc
        test_reqrep_device_tipc
        test_reqrep_tipc
        test_router_mandatory_tipc
        test_sub_forward_tipc
        test_connect_delay_tipc
        test_shutdown_stress_tipc
        test_term_endpoint_tipc
      )
    endif()
  endif()
endif()

include_directories("${CMAKE_CURRENT_LIST_DIR}/../unity")

foreach(test ${tests})
	# target_sources not supported before CMake 3.1
		
	add_executable(${test} ${test}.cpp)
	set_target_properties(${test} PROPERTIES FOLDER "Tests/ZMQ")
		
	target_link_libraries(${test} ${TESTUTIL_LIB})
	if(RT_LIBRARY)
		target_link_libraries(${test} ${RT_LIBRARY})
	endif()

	if(CMAKE_SYSTEM_NAME MATCHES "QNX")
		target_link_libraries(${test} socket)
		target_link_libraries(${test} m)
	endif()

	if(WIN32)
		add_test(NAME ${test} WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH} COMMAND ${test})
	else()
		if (${test} MATCHES "_cap_net_admin")
			add_test(NAME ${test} COMMAND sh -c "sudo ${CAPSH_PROGRAM} --caps=cap_net_admin+eip -- -c $<TARGET_FILE:${test}>")
		else()
			add_test(NAME ${test} COMMAND ${test})
		endif()
	endif()
	set_tests_properties(${test} PROPERTIES TIMEOUT 10)
	set_tests_properties(${test} PROPERTIES SKIP_RETURN_CODE 77)


		#Check whether all tests in the current folder are present
	file(READ "${CMAKE_CURRENT_LIST_FILE}" CURRENT_LIST_FILE_CONTENT)
	file(GLOB ALL_TEST_SOURCES "test_*.cpp")
	foreach(TEST_SOURCE ${tests})
		get_filename_component(TESTNAME "${TEST_SOURCE}" NAME_WE)
		string(REGEX MATCH "${TESTNAME}" MATCH_TESTNAME "${CURRENT_LIST_FILE_CONTENT}")
		if(NOT MATCH_TESTNAME)
		message(AUTHOR_WARNING "Test '${TESTNAME}' is not known to CTest.")
		endif()
	endforeach()


endforeach()

set_tests_properties(test_heartbeats PROPERTIES TIMEOUT 60)
set_tests_properties(test_security_zap PROPERTIES TIMEOUT 60)
set_tests_properties(test_many_sockets PROPERTIES TIMEOUT 120)
