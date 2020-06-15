
add_executable(logger "main.cpp")

target_link_libraries(logger SensorFusion::sf_communication)
