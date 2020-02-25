#include "Logger.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"
#include "ClockSynchronizer.h"

using namespace SF;

/*! \brief Processor implementation that only forwards the datamsgs
*
*
*/
class DataMsgForwarding : public Processor {
public:
	void CallbackSamplingTimeOver(const Time& currentTime = Now()) override {} /*!< Callback called in each sampling time - input: time for the filter*/

	void CallbackMsgQueueEmpty(const Time& currentTime = Now()) override {}  /*!< Callback called if the DataMsgs in the queue were read */
};

SF::Logger::Logger(const std::string& filename,
	const std::vector<Reciever::PeripheryProperties>& peripheries) : Application(std::make_shared<ZMQReciever>(peripheries),
	std::make_shared<DataMsgForwarding>(), std::make_shared<SPDSender>(filename)) {}

void SF::Logger::AddPeriphery(const Reciever::PeripheryProperties & prop) {
	GetRecieverPtr()->AddPeriphery(prop);
}

void SF::Logger::AddPeripheries(const NetworkConfig & config) {
	// Add clocks to synchronise
	for (auto clock : config.clockSyncData)
		GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(clock.second);
	// Add peripheries
	for (auto periphery : config.peripheryData)
		AddPeriphery(Reciever::PeripheryProperties(OperationType::SENSOR, periphery.second.RecieverAddress(), true));
}
