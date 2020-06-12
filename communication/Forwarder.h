#pragma once
#include "spdlog/logger.h"
#include <zmq.hpp>
#include "DataMsg.h"

namespace SF {

	/* Class for general methods related to forwarding DataMsg-s into log and/or ZMQ sockets
	*
	*
	*
	*/
	class Forwarder {
		// ZMQout
		std::shared_ptr<zmq::socket_t> zmq_socket;
		std::shared_ptr<zmq::context_t> zmq_context;

		// SPDlog
		std::shared_ptr<spdlog::logger> spd_logger;
		spdlog::memory_buf_t spd_buf;

	public:
		Forwarder();

		~Forwarder();

		void SetLogger(const std::string& filename);

		void SetZMQOutput(const std::string & address, int hwm);

		void ForwardDataMsg(const DataMsg& msg, const Time& currentTime);

		void ForwardString(const std::string& msg, const Time& currentTime);
	};
}
