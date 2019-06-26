#pragma once
#include "msg2buf.h"
#include <zmq.hpp>

class ZMQPublisher {
	zmq::context_t context;
	zmq::socket_t socket;

public:
	ZMQPublisher(int port);
	~ZMQPublisher() {}

	void SendString();

	void SendMsg(const DataMsg & data);
};

