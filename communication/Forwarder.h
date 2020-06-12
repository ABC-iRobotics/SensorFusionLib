#pragma once
#include "spdlog/logger.h"
#include <zmq.hpp>
#include "DataMsg.h"

namespace SF {

	/*! \brief Class for general methods related to forwarding DataMsg-s into log and/or ZMQ sockets
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
		Forwarder(); //!< Constructor

		~Forwarder(); //!< Destructor

		void SetLogger(const std::string& filename); //!< Set logfile to forward into

		void SetZMQOutput(const std::string & address, int hwm); //!< Set ZMQ socket to forward the data into

		void ForwardDataMsg(const DataMsg& msg, const Time& currentTime); //!< Forward DataMsg to the set channels

		void ForwardString(const std::string& msg, const Time& currentTime); //!< Forward string to the set channels
	};
}
