#pragma once
#include "DataMsg.h"
#include <zmq.hpp>

/*! \brief Wrapper for zmq subscriber
*
* The class initializes a zmq context and a subscriber socket (tcp://*:port)
*
* TODO: poller, drop automatically the older msg-s from the same source (zmq does not support this function)
* https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
*/
class ZMQSubscriber {
	zmq::context_t context;
	zmq::socket_t socket;

public:
	ZMQSubscriber(int port); /*!< Constructor: initializes a zmq context and a subscriber socket (tcp:// *:port) */
	~ZMQSubscriber(); /*!< Destructor */
	
	bool RecvMsg_Wait(DataMsg& data, int waitinMS=500); /*!< Wait the given time to get a msg, if it arrives, convert it to DataMsg */

	bool RecvMsg_DontWait(DataMsg& data); /*!< Check if a msg has arrived, if it has, convert it to DataMsg */

	bool RecvString_DontWait(); /*!< Check if a msg has arrived, if it has, then print it */
};

