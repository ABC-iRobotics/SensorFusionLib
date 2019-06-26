#pragma once

#include "FilterLog.h"
#include "ZMQPublisher.h"

class ZMQFilterLogger : public FilterLog {
	ZMQPublisher zmqPub;

	void Callback(const DataMsg& data) override {
		zmqPub.SendMsg(data);
	}

public:
	ZMQFilterLogger(SystemManager& filter, int port);

	~ZMQFilterLogger();
};

