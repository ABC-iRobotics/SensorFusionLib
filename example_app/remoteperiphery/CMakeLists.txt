add_executable(remoteperiphery "main.cpp")

target_link_libraries(remoteperiphery SensorFusion::sf_communication)
