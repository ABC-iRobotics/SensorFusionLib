#pragma once
#include <zmq.hpp>
#include"Application.h"

namespace SF {

	/*! \brief Sender implementation for zmq publisher
	*
	* The class initializes a zmq context and a publisher socket (tcp://localhost:port)
	*/
	class SenderToZMQ : public Sender {
		zmq::context_t context;
		zmq::socket_t socket;

	public:
		SenderToZMQ(const std::string& address); /*!< Constructor: initializes a zmq context and a publisher socket (tcp://localhost:port or ipc:///tmp/feeds/0 ...)*/

		SenderToZMQ(const SenderToZMQ&) = delete;

		void SendDataMsg(const DataMsg& data) override;

		void SendString(const std::string& str) override;
	};
}
