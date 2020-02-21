#include "CentralUnit.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"

using namespace SF;

SF::CentralUnit::CentralUnit(Processor::ProcessorPtr processor, const std::vector<Reciever::PeripheryProperties>& peripheries) :
	Application(std::make_shared<ZMQReciever>(peripheries), processor, NULL) {}

SF::CentralUnit::CentralUnit(const std::string & logfilename, Processor::ProcessorPtr processor, const std::vector<Reciever::PeripheryProperties>& peripheries) :
	Application(std::make_shared<ZMQReciever>(peripheries), processor,
		std::make_shared<SPDSender>(logfilename)) {}

SF::CentralUnit::CentralUnit(const std::string & socketaddressforresults, int hwm, Processor::ProcessorPtr processor, const std::vector<Reciever::PeripheryProperties>& peripheries) :
	Application(std::make_shared<ZMQReciever>(peripheries), processor,
		std::make_shared<ZMQSender>(socketaddressforresults, hwm)) {}

void SF::CentralUnit::AddPeriphery(const Reciever::PeripheryProperties & prop) {
	GetRecieverPtr()->AddPeriphery(prop);
}

void SF::CentralUnit::Start(DTime Ts) {
	GetRecieverPtr()->Start(Ts);
}

void SF::CentralUnit::Stop() {
	GetRecieverPtr()->Stop();
}

SF::CentralUnit::~CentralUnit() {
	Stop();
}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, Processor::ProcessorPtr processor, const std::string & inputlogfilename) :
	Application(std::make_shared<SPDReciever>(inputlogfilename, realtime), //TODO!!!
		processor, NULL) {}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, const std::string & outputlogfilename, Processor::ProcessorPtr processor,
	const std::string & inputlogfilename) :
	Application(std::make_shared<SPDReciever>(inputlogfilename, realtime),
		processor, std::make_shared<SPDSender>(outputlogfilename)) {}

SF::CentralUnitEmulator::CentralUnitEmulator(bool realtime, const std::string & socketaddressforresults, int hwm,
	Processor::ProcessorPtr processor, const std::string & inputlogfilename) :
	Application(std::make_shared<SPDReciever>(inputlogfilename, realtime),
		processor, std::make_shared<ZMQSender>(socketaddressforresults, hwm)) {}

void SF::CentralUnitEmulator::Start(DTime Ts) {
	GetRecieverPtr()->Start(Ts);
}

void SF::CentralUnitEmulator::Stop() {
	GetRecieverPtr()->Stop();
}

SF::CentralUnitEmulator::~CentralUnitEmulator() {
	//Stop();
}
