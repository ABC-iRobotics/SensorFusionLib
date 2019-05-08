#include "Filterlog.h"

FilterLog::FilterLog(SystemManager & filter) : iID(getUID()) {
	// set callback to the filter
	filter.AddCallback([this](const FilterCallData& data,
		FilterCallType event_) { Callback(std::move(data), std::move(event_)); }, iID);
	// save callback to delete the callback from the filter
	destructorCallback = [&filter, this]() { filter.DeleteCallback(iID); };
}

FilterLog::~FilterLog() {
	destructorCallback();
}
