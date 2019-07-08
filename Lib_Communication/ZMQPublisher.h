#pragma once
#include "DataMsg.h"
#include <zmq.hpp>

/*! \brief Wrapper for zmq publisher
*
* The class initializes a zmq context and a publisher socket (tcp://localhost:port)
*/
class ZMQPublisher {
	zmq::context_t context;
	zmq::socket_t socket;

public:
	ZMQPublisher(int port); /*!< Constructor: initializes a zmq context and a publisher socket (tcp://localhost:port)*/
	~ZMQPublisher(); /*!< Destructor */

	void SendString(); /*!< Send a string - for testing purposes*/

	void SendMsg(const DataMsg & data); /*!< Send a DataMsg instance */
};

