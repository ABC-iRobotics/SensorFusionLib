#pragma once

#include "ZMQSubscriber.h"
#include "ZMQPublisher.h"
#include "SystemManager.h"
#include <vector>

class FilteringManager {
	ZMQSubscriber zmqSub;
	ZMQPublisher* zmqPub;

	std::vector<System::SystemPtr> systems;

	const double Ts_max; // in seconds

protected:
	SystemManager::SystemManagerPtr filter;

	void _addSystem(System::SystemPtr systemptr);

	void _processMSG(const DataMsg & data);

	bool _getandprocessMsg_Wait(unsigned int waitinms=50) {
		DataMsg msg;
		bool success = zmqSub.RecvMsg_Wait(msg,waitinms);
		if (success)
			_processMSG(msg);
		return success;
	}

	bool _getandprocessMsg_DontWait() {
		DataMsg msg;
		bool success = zmqSub.RecvMsg_DontWait(msg);
		if (success)
			_processMSG(msg);
		return success;
	}

	FilteringManager(double Ts_max_s, int port) :
		zmqPub(NULL), Ts_max(Ts_max_s), zmqSub(port),
		systems(std::vector<System::SystemPtr>()) {}

	void SetZMQLogger(int port) {
		zmqPub = new ZMQPublisher(port);
		filter->SetCallback([this](const DataMsg& data) {
			zmqPub->SendMsg(data);
		});
	}

public:

	void run() {
		TimeMicroSec t_last;
		// törölni minden várakozó üzenetet?

		while (true) {
			// Várunk valamennyi ideig üzenetre // TODO: poll?, intelligensebb kiolvasás?, a kiolvasás sorrendje, timestamp használata?
			bool msgs = _getandprocessMsg_Wait((unsigned int)(Ts_max * 1000));
			while (msgs) // ha van, megnézzük van-e még
				msgs = _getandprocessMsg_DontWait();
			TimeMicroSec t_filt;
			filter->Step(t_last - t_filt); // gyorsan léptetni a fúziót - az eltelt idõnek megfelelõen...
			t_last = t_filt;
		}
	}

	~FilteringManager() {
		if (!zmqPub)
			delete zmqPub;
	}
};
