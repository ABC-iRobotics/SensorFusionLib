#include "FilteringManager.h"

void FilteringManager::_processMSG(const DataMsg & data) {
	if (data.GetDataSourceType() == SENSOR)
		filter->SetProperty(data);
}

bool FilteringManager::_getandprocessMsg_Wait(unsigned int waitinms) {
	DataMsg msg;
	bool success = zmqSub.RecvMsg_Wait(msg, waitinms);
	if (success)
		_processMSG(msg);
	return success;
}

bool FilteringManager::_getandprocessMsg_DontWait() {
	DataMsg msg;
	bool success = zmqSub.RecvMsg_DontWait(msg);
	if (success)
		_processMSG(msg);
	return success;
}

FilteringManager::FilteringManager(double Ts_max_s, int port) :
	zmqPub(NULL), Ts_max(Ts_max_s), zmqSub(port), filter(NULL) {}

void FilteringManager::SetFilter(SystemManager::SystemManagerPtr filter_) {
	filter = filter_;
}

void FilteringManager::SetZMQLogger(int port) {
	zmqPub = new ZMQPublisher(port);
	filter->SetCallback([this](const DataMsg& data) {
		zmqPub->SendMsg(data);
	});
}

void FilteringManager::run() {
	TimeMicroSec t_last;
	// t�r�lni minden v�rakoz� �zenetet?

	while (filter) {
		// V�runk valamennyi ideig �zenetre // TODO: poll?, intelligensebb kiolvas�s?, a kiolvas�s sorrendje, timestamp haszn�lata?
		bool msgs = _getandprocessMsg_Wait((unsigned int)(Ts_max * 1000));
		while (msgs) // ha van, megn�zz�k van-e m�g
			msgs = _getandprocessMsg_DontWait();
		TimeMicroSec t_filt;
		filter->Step(t_filt - t_last); // gyorsan l�ptetni a f�zi�t - az eltelt id�nek megfelel�en...
		t_last = t_filt;
	}
}

FilteringManager::~FilteringManager() {
	if (!zmqPub)
		delete zmqPub;
}
