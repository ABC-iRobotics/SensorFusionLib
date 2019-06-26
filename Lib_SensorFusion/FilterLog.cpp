#include "FilterLog.h"

FilterLog::FilterLog(SystemManager & filter) : iID(getUID()) {
	// set callback to the filter
	filter.AddCallback([this](const DataMsg& data) { Callback(std::move(data)); }, iID);
	// save callback to delete the callback from the filter
	destructorCallback = [&filter, this]() { filter.DeleteCallback(iID); };
}

FilterLog::~FilterLog() {
	destructorCallback();
}
