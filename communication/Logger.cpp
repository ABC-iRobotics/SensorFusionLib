#include "Logger.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"

using namespace SF;

SF::Logger::Logger(const std::string& filename,
	const std::vector<Reciever::PeripheryProperties>& peripheries) : Application(std::make_shared<ZMQReciever>(peripheries),
	std::make_shared<Processor>(), std::make_shared<SPDSender>(filename, "Logger")) {}

void SF::Logger::AddPeriphery(const Reciever::PeripheryProperties & prop) {
	GetRecieverPtr()->AddPeriphery(prop);
}
