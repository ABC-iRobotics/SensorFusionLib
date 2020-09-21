#include "Logger.h"

using namespace SF;

SF::Logger::Logger(const std::string& filename) : Forwarder(), ZMQReciever() {
	SetLogger(filename);
}

/*!< Must called if new DataMsg recieved */

bool SF::Logger::SaveDataMsg(const DataMsg & msg, const Time & currentTime) {
	ForwardDataMsg(msg, currentTime);
	return true;
}

void SF::Logger::SaveString(const std::string & msg, const Time & currentTime) {
	ForwardString(msg, currentTime);
}
