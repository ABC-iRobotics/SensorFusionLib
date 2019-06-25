#pragma once

#include "FilterLog.h"
#include "ZMQPublisher.h"

class ZMQFilterLogger : public FilterLog {
	ZMQPublisher zmqPub;

	void Callback(const FilterCallData& data) override {
		SystemDataMsg::ContentTypes type;
		switch (data.type)
		{
		case STATE:
			if (data.callType == FilterCallData::PREDICTION)
				type = SystemDataMsg::FROMFILTER_PREDICTEDSTATE;
			else
				type = SystemDataMsg::FROMFILTER_FILTEREDSTATE;
			break;
		case OUTPUT:
			if (data.callType == FilterCallData::PREDICTION)
				type = SystemDataMsg::FROMFILTER_PREDICTEDOUTPUT;
			if (data.callType == FilterCallData::MEASUREMENT)
				type = SystemDataMsg::FROMFILTER_MEASUREDOUTPUT;
			break;
		case DISTURBANCE:
			type = SystemDataMsg::FROMFILTER_USEDDISTURBANCE;
			break;
		case NOISE:
			type = SystemDataMsg::FROMFILTER_USEDNOISE;
			break;
		}
		SystemDataMsg msg(0, type, data.t*1e6);
		msg.SetValueVector(data.value.vector);
		msg.SetVarianceMatrix(data.value.variance);
		zmqPub.SendMsg(msg);
	}

public:
	ZMQFilterLogger(SystemManager& filter, int port);

	~ZMQFilterLogger();
};

