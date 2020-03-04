add_executable(periphery "main.cpp")

target_link_libraries(periphery SensorFusion::sf_communication)
