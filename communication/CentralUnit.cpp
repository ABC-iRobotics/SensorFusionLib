#include "CentralUnit.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"
#include "ClockSynchronizer.h"

using namespace SF;

SF::CentralUnit::CentralUnit(AppLayer::AppLayerPtr processor_, const NetworkConfig& config) :
	reciever(std::make_shared<ZMQReciever>()), sender(NULL), processor(processor_) {
	reciever->AddNextLayer(processor);
	std::dynamic_pointer_cast<ZMQReciever>(reciever)->AddPeripheries(config);
}

SF::CentralUnit::CentralUnit(const std::string & logfilename, AppLayer::AppLayerPtr processor_,
	const NetworkConfig& config) :
	reciever(std::make_shared<ZMQReciever>()), sender(std::make_shared<SPDSender>(logfilename)), processor(processor_) {
	reciever->AddNextLayer(processor);
	processor->AddNextLayer(sender);
	std::dynamic_pointer_cast<ZMQReciever>(reciever)->AddPeripheries(config);
}

SF::CentralUnit::CentralUnit(const std::string & socketaddressforresults, int hwm,
	AppLayer::AppLayerPtr processor_, const NetworkConfig& config) :
	reciever(std::make_shared<ZMQReciever>()), sender(std::make_shared<ZMQSender>(socketaddressforresults, hwm)) {
	reciever->AddNextLayer(processor);
	processor->AddNextLayer(sender);
	std::dynamic_pointer_cast<ZMQReciever>(reciever)->AddPeripheries(config);
}

SF::CentralUnit::CentralUnit(Sender::SenderPtr sender_, AppLayer::AppLayerPtr processor_,
	const NetworkConfig& config) :
	reciever(std::make_shared<ZMQReciever>()), sender(sender_) {
	reciever->AddNextLayer(processor_);
	processor->AddNextLayer(sender);
	std::dynamic_pointer_cast<ZMQReciever>(reciever)->AddPeripheries(config);
}

void SF::CentralUnit::Start(DTime Ts) {
	reciever->Start(Ts);
}

void SF::CentralUnit::Stop() {
	reciever->Stop();
}

SF::CentralUnit::~CentralUnit() {
	reciever->Stop();
}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, AppLayer::AppLayerPtr processor_, const std::string & inputlogfilename) :
	reciever(std::make_shared<SPDReciever>(inputlogfilename, realtime)), processor(processor_) {
	reciever->AddNextLayer(processor_);
	processor->AddNextLayer(sender);
}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, const std::string & outputlogfilename, AppLayer::AppLayerPtr processor_,
	const std::string & inputlogfilename) :
	reciever(std::make_shared<SPDReciever>(inputlogfilename, realtime)), sender(std::make_shared<SPDSender>(outputlogfilename)), processor(processor_) {
	reciever->AddNextLayer(processor);
	processor->AddNextLayer(sender);
}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, const std::string & socketaddressforresults, int hwm,
	AppLayer::AppLayerPtr processor_, const std::string & inputlogfilename) :
	reciever(std::make_shared<SPDReciever>(inputlogfilename, realtime)), sender(std::make_shared<ZMQSender>(socketaddressforresults, hwm)),
	processor(processor_) {
	reciever->AddNextLayer(processor);
	processor->AddNextLayer(sender);
}

void SF::CentralUnitEmulator::Start(DTime Ts) {
	reciever->Start(Ts);
}

void SF::CentralUnitEmulator::Stop() {
	reciever->Stop();
}

SF::CentralUnitEmulator::~CentralUnitEmulator() {
	Stop();
}

bool SF::CentralUnitEmulator::isRunning() {
	return reciever->IsRunning();
}
