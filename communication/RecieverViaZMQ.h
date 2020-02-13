#pragma once
#include <zmq.hpp>
#include "Application.h"

namespace SF {

	/*! \brief Reciever implementation for ZMQ based communication
	*
	*/
	class RecieverViaZMQ : public Reciever {

		zmq::context_t context;
		zmq::socket_t socket;
		std::mutex socketguard;
		std::thread subscriberThread;
		bool toStop;

		void Run(DTime Ts);

	public:
		RecieverViaZMQ();  /*!< Constructor */

		void Start(DTime Ts) override;

		void Stop(bool waitin = true) override;

		void ConnectToAddress(const std::string& address) override;
	};

}
