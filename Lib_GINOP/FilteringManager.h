#pragma once

#include "ZMQSubscriber.h"
#include "ZMQPublisher.h"
#include "SystemManager.h"
#include "ZMQFilterLogger.h"
#include <map>

class FilteringManager {
	ZMQSubscriber zmqSub;

	ZMQFilterLogger* zmqLogger;

	std::map<unsigned int, System::SystemPtr> systems; // ID - Systems

	const double Ts_max; // in seconds

protected:
	SystemManager::SystemManagerPtr filter;

	void _addSystem(unsigned int zmqID, System::SystemPtr systemptr);

	void _processMSG(const SystemDataMsg& msg);

	bool _getandprocessMsg_Wait(unsigned int waitinms=50) {
		SystemDataMsg msg;
		bool success = zmqSub.RecvMsg_Wait(msg,waitinms);
		if (success)
			_processMSG(msg);
		return success;
	}

	bool _getandprocessMsg_DontWait() {
		SystemDataMsg msg;
		bool success = zmqSub.RecvMsg_DontWait(msg);
		if (success)
			_processMSG(msg);
		return success;
	}

	FilteringManager(double Ts_max_s, int port);

	void SetZMQLogger(SystemManager& filter, int port) {
		zmqLogger = new ZMQFilterLogger(filter, port);
	}

public:

	void run() {
		unsigned long t_last = getTimeInMicroseconds();
		// törölni minden várakozó üzenetet?

		while (true) {
			// Várunk valamennyi ideig üzenetre // TODO: poll?, intelligensebb kiolvasás?, a kiolvasás sorrendje, timestamp használata?
			bool msgs = _getandprocessMsg_Wait((unsigned int)(Ts_max * 1000));
			while (msgs) // ha van, megnézzük van-e még
				msgs = _getandprocessMsg_DontWait();
			unsigned long t_filt = getTimeInMicroseconds();
			filter->Step(double(t_last - t_filt) / 1e6); // gyorsan léptetni a fúziót - az eltelt idõnek megfelelõen...
			t_last = t_filt;
		}
	}

	~FilteringManager() {
		if (!zmqLogger)
			delete zmqLogger;
	}

	// ráregisztrálni callback-et a szûrõre, h továbbítsa az adatokat zmq-n?

	// log?

	// külön init
	// start szál, ami fut magának...

	// megjelenítés blokkolás nélkül?
};
