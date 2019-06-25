#include "ZMQPublisher.h"
#include <string>
#include <iostream>
#include <thread>

ZMQPublisher::ZMQPublisher(int port) : context(1), socket(context, ZMQ_PUB) {
	std::string protocol = "tcp://*:" + std::to_string(port);
	socket.bind(protocol.c_str());
}

void ZMQPublisher::SendString() {
	const char str[50] = "Mukodes teszt...";
	zmq::message_t request((void*)str, 50, NULL);
	std::cout << "Sent: " << request.size() << " byte. [1]: " << static_cast<char*>(request.data())[1] << " \n\n";
	socket.send(request, zmq::send_flags::none);
	std::this_thread::sleep_for(std::chrono::duration<float, std::micro>(5));
}

void ZMQPublisher::SendMsg(const SystemDataMsg & data) {
	Buffer b = data.GetMsgBuffer();
	zmq::message_t request((void*)b.Buf(), b.Size(), NULL);
	socket.send(request, zmq::send_flags::none);
	std::this_thread::sleep_for(std::chrono::duration<float, std::micro>(50)); // 5 us is enough for messages of 50 byte
}
