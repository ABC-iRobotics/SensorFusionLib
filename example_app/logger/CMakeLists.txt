
file(COPY ${CMAKE_SOURCE_DIR}/networkconfig_1.json
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(logger "main.cpp")

target_link_libraries(logger SensorFusion::sf_communication)
