#include "FilteringManager.h"

void FilteringManager::_addSystem(System::SystemPtr systemptr) {
	systems.push_back(systemptr);
}

void FilteringManager::_processMSG(const DataMsg & data) {
	if (data.GetDataSourceType() == SENSOR)
		filter->SetProperty(data);
}
