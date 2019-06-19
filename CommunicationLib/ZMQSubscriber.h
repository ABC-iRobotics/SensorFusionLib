#pragma once

#include "msg2buf.h"
#include <zmq.hpp>

class ZMQSubscriber {
	zmq::context_t context;
	zmq::socket_t socket;

public:
	ZMQSubscriber();
	~ZMQSubscriber();
	
	bool RecvMsg_Wait(DataMsg& data, int waitinMS=500);

	bool RecvMsg_DontWait(DataMsg& data);

	// TODO: Poller
	//https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait

	// TODO: howto drop automatically older msges of a source

	bool RecvString_DontWait();
};

