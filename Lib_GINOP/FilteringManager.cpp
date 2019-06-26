#include "FilteringManager.h"

void FilteringManager::_addSystem(System::SystemPtr systemptr) {
	systems.push_back(systemptr);
}

void FilteringManager::_processMSG(const DataMsg & data) {
	if (data.GetDataSourceType() == SENSOR)
		filter->SetProperty(data);
}

FilteringManager::FilteringManager(double Ts_max_s, int port) :
	zmqLogger(NULL), Ts_max(Ts_max_s), zmqSub(port),
	systems(std::vector<System::SystemPtr>()) {}
