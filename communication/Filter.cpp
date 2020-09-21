#include "Filter.h"

using namespace SF;

SF::Filter::Filter(FilterCore::FilterCorePtr filterCore_)
	: Forwarder(), ZMQReciever(), filterCore(filterCore_) {}

/*!< Must called if new DataMsg recieved */


/*!< Must called in each sampling time - input: time */

void SF::Filter::SamplingTimeOver(const Time & currentTime) {
	filterCore->SamplingTimeOver(currentTime);
	// forward filtered state
	for (int i = 0; i < filterCore->nSensors() + 1; i++)
		ForwardDataMsg(filterCore->GetDataByIndex(i - 1, DataType::STATE, OperationType::FILTER_MEAS_UPDATE), currentTime);
}

void SF::Filter::SaveDataMsg(const DataMsg & msg, const Time & currentTime) {
	filterCore->SaveDataMsg(msg, currentTime);
	ForwardDataMsg(msg, currentTime);
}

/*!< Must called if the DataMsgs in the queue were read */

void SF::Filter::MsgQueueEmpty(const Time & currentTime) {
	filterCore->MsgQueueEmpty(currentTime);
}

void SF::Filter::SaveString(const std::string & msg, const Time & currentTime) {
	ForwardString(msg, currentTime);
}
