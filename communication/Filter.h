#pragma once
#include "FilterCore.h"
#include "Forwarder.h"
#include "ZMQReciever.h"

namespace SF {

	/*! \brief Class for realtime filtering
	*
	* The data is collected from zmq by inheriting the methods of class ZMQReciever
	*
	* The filtering is done via a FilterCore instance set in the constructor
	*
	* The results are forwarded via inherited methods of class Forwarder
	*/
	class Filter : public Forwarder, public ZMQReciever {
		using Forwarder::ForwardDataMsg;

		using Forwarder::ForwardString;

		FilterCore::FilterCorePtr filterCore;

	public:
		Filter(FilterCore::FilterCorePtr filterCore_);

	protected:
		/*!< Must called in each sampling time - input: time */
		void SamplingTimeOver(const Time& currentTime) override;

		/*!< Must called if new DataMsg recieved */
		bool SaveDataMsg(const DataMsg& msg, const Time& currentTime) override;

		/*!< Must called if the DataMsgs in the queue were read */
		void MsgQueueEmpty(const Time& currentTime) override;

		void SaveString(const std::string& msg, const Time& currentTime) override;
	};
}
