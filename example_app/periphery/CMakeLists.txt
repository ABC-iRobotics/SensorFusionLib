
file(COPY ${CMAKE_SOURCE_DIR}/networkconfig_1.json
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(periphery "main.cpp")

target_link_libraries(periphery SensorFusion::sf_communication)
