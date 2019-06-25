#include "FilteringManager.h"

void FilteringManager::_addSystem(unsigned int zmqID, System::SystemPtr systemptr) {
	systems.insert(std::pair<unsigned int, System::SystemPtr>(zmqID, systemptr));
}

void FilteringManager::_processMSG(const SystemDataMsg & msg) {
	if (!msg.IsToFilter()) {
		auto it = systems.find(msg.SourceID());
		if (it != systems.end()) {
			System::SystemPtr system = it->second;
			switch (msg.ContentType()) {
			case SystemDataMsg::TOFILTER_MEASUREMENT:
				if (msg.HasValue())
					system->MeasurementDone(msg.Value());
				if (msg.HasVariance())
					system->SetNoiseVariance(msg.Variance());
				break;
			case SystemDataMsg::TOFILTER_DISTURBANCE:
				if (msg.HasValue())
					system->SetDisturbanceValue(msg.Value());
				if (msg.HasVariance())
					system->SetDisturbanceVariance(msg.Variance());
				break;
			}
		}
	}
}

FilteringManager::FilteringManager(double Ts_max_s, int port) : zmqLogger(NULL), Ts_max(Ts_max_s), zmqSub(port), systems(std::map<unsigned int, System::SystemPtr>()) {}
