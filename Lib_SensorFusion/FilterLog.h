#pragma once

#include "SystemManager.h"

class FilterLog {
public:
	FilterLog(SystemManager& filter);

	virtual void Callback(const FilterCallData& data) = 0;

	~FilterLog();

private:
	unsigned int iID;

	std::function<void(void)> destructorCallback;
};

