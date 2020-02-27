add_executable(localperiphery "main.cpp")

target_link_libraries(localperiphery SensorFusion::sf_communication)
