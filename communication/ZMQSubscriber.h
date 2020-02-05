#pragma once
#include "DataMsg.h"
#include <zmq.hpp>
#include <vector>

namespace SF {

	/*! \brief Wrapper for zmq subscriber
	*
	* The class initializes a zmq context and a subscriber socket (tcp://*:port)
	*
	* TODO: poller, drop automatically the older msg-s from the same source (zmq does not support this function)
	* https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
	*/
	class ZMQSubscriber {
	protected:
		zmq::context_t context;
		std::vector<zmq::socket_t*> sockets;
	public:
		size_t numSockets() const { return sockets.size(); }

		virtual void addSocket(std::string address); // e.g. "tcp://10.8.0.14:5555" or "ipc:///tmp/feeds/0"

		ZMQSubscriber() : context(1), sockets(std::vector<zmq::socket_t*>()) {}

		~ZMQSubscriber() {
			for (int i = 0; i < sockets.size(); i++)
				sockets[i]->close();
			for (int i = 0; i < sockets.size(); i++)
				delete sockets[i];
		}

		ZMQSubscriber(const ZMQSubscriber&) = delete;
	};


	class ZMQRTSubscriber : public ZMQSubscriber {
		std::vector<zmq::message_t> lastData;
		std::vector<bool> novelty;

	public:
		ZMQRTSubscriber(); /*!< Constructor: initializes a zmq context and a subscriber socket (tcp:// *:port) */

		ZMQRTSubscriber(const ZMQRTSubscriber&) = delete;

		void addSocket(std::string address) override; // e.g. "tcp://10.8.0.14:5555" or "ipc:///tmp/feeds/0"

		bool RecvMsg_Wait(int waitinMS = 500); /*!< Wait the given time to get a msg, if it arrives, convert it to DataMsg */

		bool RecvMsg_DontWait(); /*!< Check if a msg has arrived, if it has, convert it to DataMsg */

		void getData(int index, DataMsg& data);

		void RecvString_DontWait(); /*!< Check if a msg has arrived, if it has, then print it */
	};

	class ZMQLogSubscriber : public ZMQSubscriber {
	public:
		ZMQLogSubscriber() {} //Constructor

		ZMQLogSubscriber(const ZMQLogSubscriber&) = delete;

		bool RecvMsg_DontWait(DataMsg& data); /*!< Check if a msg has arrived, if it has, convert it to DataMsg */

		bool RecvMsg_Wait(DataMsg& data, int waitinMS = 500); /*!< Wait the given time to get a msg, if it arrives, convert it to DataMsg */

		void RecvString_DontWait(); /*!< Check if a msg has arrived, if it has, then print it */
	};

}
