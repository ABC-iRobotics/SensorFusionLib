#include "Logger.h"
#include "SPDLogging.h"
#include "ZMQCommunication.h"

using namespace SF;

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
