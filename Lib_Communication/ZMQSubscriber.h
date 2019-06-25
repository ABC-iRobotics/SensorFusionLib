#pragma once

#include "msg2buf.h"
#include <zmq.hpp>

class ZMQSubscriber {
	zmq::context_t context;
	zmq::socket_t socket;

public:
	ZMQSubscriber(int port);
	~ZMQSubscriber();
	
	bool RecvMsg_Wait(SystemDataMsg& data, int waitinMS=500);

	bool RecvMsg_DontWait(SystemDataMsg& data);

	// TODO: Poller
	//https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait

	// TODO: howto drop automatically older msges of a source

	bool RecvString_DontWait();
};

