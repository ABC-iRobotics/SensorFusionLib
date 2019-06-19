#include "ZMQSubscriber.h"
#include <string>
#include <iostream>

ZMQSubscriber::ZMQSubscriber() : context(1), socket(context, ZMQ_SUB) {
	std::string protocol = "tcp://localhost:" + std::to_string(port);
	socket.connect(protocol);
	socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
}

ZMQSubscriber::~ZMQSubscriber() {
	socket.close();
}

bool ZMQSubscriber::RecvMsg_Wait(DataMsg & data, int waitinMS) {
	socket.setsockopt(ZMQ_RCVTIMEO, waitinMS);
	zmq::message_t reply;
	auto success = socket.recv(reply, zmq::recv_flags::none);
	if (success.has_value()) {
		Buffer b = Buffer(static_cast<unsigned char*>(reply.data()), reply.size());
		data = DataMsg(b);
	}
	else
		data = DataMsg();
	return success.has_value();
}

bool ZMQSubscriber::RecvMsg_DontWait(DataMsg & data) {
	zmq::message_t reply;
	auto success = socket.recv(reply, zmq::recv_flags::dontwait);
	if (success.has_value()) {
		Buffer b = Buffer(static_cast<unsigned char*>(reply.data()), reply.size());
		data = DataMsg(b);
	}
	else
		data = DataMsg();
	return success.has_value();
}

bool ZMQSubscriber::RecvString_DontWait() {
	zmq::message_t reply;
	auto success = socket.recv(reply, zmq::recv_flags::dontwait);
	if (success.has_value()) {
		std::cout << "Got: " << reply.size() << "byte: " << static_cast<char*>(reply.data()) << std::endl;
	}
	return success.has_value();
}
