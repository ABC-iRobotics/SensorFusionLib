#include "Logger.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"
#include "ClockSynchronizer.h"

using namespace SF;

SF::Logger::Logger(const std::string& filename) :
	sender(std::make_shared<SPDSender>(filename)),
	reciever(std::make_shared<ZMQReciever>()) {
	reciever->AddNextLayer(sender);
}

void SF::Logger::AddPeripheries(const NetworkConfig & config) {
	std::dynamic_pointer_cast<ZMQReciever>(reciever)->AddPeripheries(config);
}

void SF::Logger::Start(DTime Ts) {
	reciever->Start(Ts);
}

void SF::Logger::Stop() {
	reciever->Stop();
}

SF::Logger::~Logger() {
	reciever->Stop();
}
