#pragma once
#include "Forwarder.h"
#include "ZMQReciever.h"

namespace SF {
	
	/*! \brief Class for recieveing msgs-s from ZMQ and save them into a log
	*
	* The data is collected from zmq by inheriting the methods of class ZMQReciever
	*
	* The results are saved via inherited methods of class Forwarder
	*/
	class Logger : private Forwarder, public ZMQReciever {

		using Forwarder::ForwardDataMsg;

		using Forwarder::ForwardString;

		using Forwarder::SetZMQOutput;

	public:
		Logger(const std::string& filename);


	protected:
		/*!< Must called in each sampling time - input: time */
		void SamplingTimeOver(const Time& currentTime) override {}

		/*!< Must called if new DataMsg recieved */
		void SaveDataMsg(const DataMsg& msg, const Time& currentTime) override;

		/*!< Must called if the DataMsgs in the queue were read */
		void MsgQueueEmpty(const Time& currentTime) override {}

		void SaveString(const std::string& msg, const Time& currentTime) override;
	};
}
