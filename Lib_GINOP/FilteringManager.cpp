#include "FilteringManager.h"

void FilteringManager::_processMSG(const DataMsg & data) {
	if (data.GetDataSourceType() == SENSOR)
		filter->SetProperty(data);
}

bool FilteringManager::_getandprocessMsg_Wait(unsigned int waitinms) {
	bool success = zmqSub.RecvMsg_Wait(waitinms);
	if (success)
		_readLoadedZMQMsgs();
	return success;
}

bool FilteringManager::_getandprocessMsg_DontWait() {
	bool success = zmqSub.RecvMsg_DontWait();
	if (success)
		_readLoadedZMQMsgs();
	return success;
}

void FilteringManager::_readLoadedZMQMsgs() {
	DataMsg msg;
	for (unsigned int i = 0; i < zmqSub.numSockets(); i++) {
		zmqSub.getData(i, msg);
		if (!msg.IsEmpty())
			_processMSG(msg);
	}
}

FilteringManager::FilteringManager(double Ts_max_s) :
	zmqPub(NULL), Ts_max(Ts_max_s), zmqSub(), filter(NULL) {}

void FilteringManager::addSensorSockets(std::string address) {
	zmqSub.addSocket(address);
}

void FilteringManager::SetFilter(SystemManager::SystemManagerPtr filter_) {
	filter = filter_;
}

void FilteringManager::SetZMQLogger(std::string address) {
	zmqPub = new ZMQPublisher(address);
	filter->SetCallback([this](const DataMsg& data) {
		zmqPub->SendMsg(data);
	});
}

void FilteringManager::run() {
	TimeMicroSec t_last;
	// törölni minden várakozó üzenetet?

	while (filter) {
		// Várunk valamennyi ideig üzenetre // TODO: poll?, intelligensebb kiolvasás?, a kiolvasás sorrendje, timestamp használata?
		bool msgs = _getandprocessMsg_Wait((unsigned int)(Ts_max * 1000));
		while (msgs) // ha van, megnézzük van-e még
			msgs = _getandprocessMsg_DontWait();
		TimeMicroSec t_filt;
		filter->Step(t_filt - t_last); // gyorsan léptetni a fúziót - az eltelt idõnek megfelelõen...
		t_last = t_filt;
	}
}

FilteringManager::~FilteringManager() {
	if (!zmqPub)
		delete zmqPub;
}
