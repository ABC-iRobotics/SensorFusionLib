add_executable(periphery_w_clocksync "main.cpp")

target_link_libraries(periphery_w_clocksync SensorFusion::sf_communication)
