#include "SenderToZMQ.h"
#include "msg2buf.h"
#include <string>
#include <iostream>
#include <thread>

using namespace SF;

SenderToZMQ::SenderToZMQ(const std::string& address) : context(1) {
	socket = zmq::socket_t(context, ZMQ_PUB);
	socket.bind(address.c_str());
}

void SF::SenderToZMQ::SendString(const std::string & str) {
	zmq::message_t msg((void*)str.c_str(), str.length(), NULL);
	socket.send(msg, zmq::send_flags::none);
	std::this_thread::sleep_for(std::chrono::duration<float, std::micro>(5));
}

void SenderToZMQ::SendDataMsg(const DataMsg& data) {
	Buffer b(data);
	zmq::message_t request((void*)b.Buf(), b.Size(), NULL);
	socket.send(request, zmq::send_flags::none);
	std::this_thread::sleep_for(std::chrono::duration<float, std::micro>(50)); // 5 us is enough for messages of 50 byte
}
