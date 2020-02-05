#include "ZMQSubscriber.h"
#include "msg2buf.h"
#include <string>
#include <iostream>
#include <thread>

using namespace SF;

ZMQRTSubscriber::ZMQRTSubscriber() : 
	lastData(std::vector<zmq::message_t>()), novelty(std::vector<bool>()) {}

void ZMQSubscriber::addSocket(std::string address) {
	zmq::socket_t *newsocket = new zmq::socket_t(context, ZMQ_SUB);
	newsocket->connect(address.c_str());
	newsocket->setsockopt(ZMQ_SUBSCRIBE, "", 0);
	newsocket->setsockopt(ZMQ_LINGER, 0); // dont store msg-s after socket is closed
	sockets.push_back(newsocket);
}

void ZMQRTSubscriber::addSocket(std::string address) {
	ZMQSubscriber::addSocket(address);
	lastData.push_back(zmq::message_t());
	novelty.push_back(false);
}

void ZMQRTSubscriber::getData(int index, DataMsg& data) {
	if (novelty[index]) {
		Buffer b = Buffer(static_cast<unsigned char*>(lastData[index].data()), lastData[index].size());
		data = b.ExtractDataMsg();
		novelty[index] = false;
	}
	else data = DataMsg();
}

bool ZMQRTSubscriber::RecvMsg_Wait(int waitinMS) {
	auto start = std::chrono::system_clock::now();
	while (waitinMS < 0 ||
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < waitinMS) {
		if (RecvMsg_DontWait())
			return true;
		std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(1));
	}
	return false;
}

bool ZMQRTSubscriber::RecvMsg_DontWait() {
	bool out = false;
	zmq::message_t reply;
	for (int i = 0; i < sockets.size(); i++) {
		while (sockets[i]->recv(reply, zmq::recv_flags::dontwait).has_value()) {
			lastData[i].copy(reply);
			novelty[i] = true;
			out = true;
		}
	}
	return out;
}

void ZMQRTSubscriber::RecvString_DontWait() {
	zmq::message_t reply;
	for (int i = 0; i < sockets.size(); i++) {
		auto success = sockets[i]->recv(reply, zmq::recv_flags::dontwait);
		if (success.has_value()) {
			std::cout << "Got: " << reply.size() << "byte: " << static_cast<char*>(reply.data()) << std::endl;
		}
	}
}

bool ZMQLogSubscriber::RecvMsg_DontWait(DataMsg& data) {
	zmq::message_t reply;
	for (int i = 0; i < sockets.size(); i++) {
		if (sockets[i]->recv(reply, zmq::recv_flags::dontwait).has_value()) {
			Buffer b = Buffer(static_cast<unsigned char*>(reply.data()), reply.size());
			data = b.ExtractDataMsg();
			return true;
		}
	}
	return false;
}

bool ZMQLogSubscriber::RecvMsg_Wait(DataMsg& data, int waitinMS) {
	auto start = std::chrono::system_clock::now();
	while (waitinMS < 0 || 
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-start).count() < waitinMS) {
		if (RecvMsg_DontWait(data))
			return true;
		std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(1));
	}
	return false;
}

void ZMQLogSubscriber::RecvString_DontWait() {
	zmq::message_t reply;
	for (int i = 0; i < sockets.size(); i++) {
		auto success = sockets[i]->recv(reply, zmq::recv_flags::dontwait);
		if (success.has_value()) {
			std::cout << "Got: " << reply.size() << "byte: " << static_cast<char*>(reply.data()) << std::endl;
		}
	}
}