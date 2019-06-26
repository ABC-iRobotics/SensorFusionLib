#pragma once

#include "FilterLog.h"
#include "ZMQPublisher.h"

class ZMQFilterLogger : public FilterLog {
	ZMQPublisher zmqPub;

	void Callback(const FilterCallData& data) override {
		OperationType source;
		switch (data.callType) {
		case FilterCallData::PREDICTION:
			source = FILTER_TIME_UPDATE;
			break;
		case FilterCallData::FILTERING:
			source = FILTER_MEAS_UPDATE;
			break;
		case FilterCallData::MEASUREMENT:
			source = SENSOR;
			break;
		case FilterCallData::ESTIMATION:
		default:
			source = FILTER_PARAM_ESTIMATION;
			break;
		}
		DataMsg msg(0, data.type, source, data.t*1e6);
		msg.SetValueVector(data.value.vector);
		msg.SetVarianceMatrix(data.value.variance);
		zmqPub.SendMsg(msg);
	}

public:
	ZMQFilterLogger(SystemManager& filter, int port);

	~ZMQFilterLogger();
};

