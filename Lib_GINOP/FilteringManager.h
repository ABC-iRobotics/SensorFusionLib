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
		// t�r�lni minden v�rakoz� �zenetet?

		while (true) {
			// V�runk valamennyi ideig �zenetre // TODO: poll?, intelligensebb kiolvas�s?, a kiolvas�s sorrendje, timestamp haszn�lata?
			bool msgs = _getandprocessMsg_Wait((unsigned int)(Ts_max * 1000));
			while (msgs) // ha van, megn�zz�k van-e m�g
				msgs = _getandprocessMsg_DontWait();
			unsigned long t_filt = getTimeInMicroseconds();
			filter->Step(double(t_last - t_filt) / 1e6); // gyorsan l�ptetni a f�zi�t - az eltelt id�nek megfelel�en...
			t_last = t_filt;
		}
	}

	~FilteringManager() {
		if (!zmqLogger)
			delete zmqLogger;
	}

	// r�regisztr�lni callback-et a sz�r�re, h tov�bb�tsa az adatokat zmq-n?

	// log?

	// k�l�n init
	// start sz�l, ami fut mag�nak...

	// megjelen�t�s blokkol�s n�lk�l?
};
