#include "ClockSynchronizer.h"
#include "defs.h"
#include <string>
#include <iostream>
#include<cstdint>
#include<cstdio>

using namespace SF;

class LittleEndianSerializer {
	enum Endianness { LITTLE_ENDIAN, BIG_ENDIAN } endianness;
public:
	LittleEndianSerializer() {
		uint32_t word = 0x0A0B0C0D; // An unsigned 32-bit integer.
		char *pointer = (char *)&word; // A pointer to the first octet of the word.
		if (pointer[0] == 0x0D) {
			endianness = LITTLE_ENDIAN;
			return;
		}
		if (pointer[0] == 0x0A) {
			endianness = BIG_ENDIAN;
			return;
		}
		perror("Unkown endianness");
	}
	void memcpy(void* dest, const void* src, int size) {
		const char *srcptr = (const char *)src;
		char *destptr = (char *)dest;
		if (endianness == LITTLE_ENDIAN)
			for (int i = 0; i < size; i++)
				destptr[i] = srcptr[i];
		if (endianness == BIG_ENDIAN)
			for (int i = 0; i < size; i++)
				destptr[size - i - 1] = srcptr[i];
	}
};

static LittleEndianSerializer littleEndianSerializer;

SF::ClockSynchronizerServer::ClockSynchronizerServer(const std::string & address) : running(false) {
	//  Prepare our context and socket
	context = zmq::context_t(1);
	socket = zmq::socket_t(context, ZMQ_REP);
	socket.bind(address);
}

void SF::ClockSynchronizerServer::StartServer() {
	if (!running) {
		running = true;
		serverthread = std::thread(&ClockSynchronizerServer::Run, this);
	}
}

void SF::ClockSynchronizerServer::StopServer() {
	if (running) {
		running = false;
		serverthread.join();
	}
}

SF::ClockSynchronizerServer::~ClockSynchronizerServer() {
	StopServer();
}

void SF::ClockSynchronizerServer::Run() {
	std::cout << "ClockSynchronizerServer started\n";
	zmq::message_t reply(8);
	zmq::message_t request;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	while (running) {
		//  Wait for next request from client with timeouts 200ms
		auto res = socket.recv(request);
		if (res.has_value()) { // && res.value() != EAGAIN) {
			long long time = duration_since_epoch(Now()).count();
			//  Send reply back to client
			littleEndianSerializer.memcpy(reply.data(), &time, 8);
			socket.send(reply, zmq::send_flags::dontwait);
			reply.rebuild(8);
			request.rebuild();
		}
	}
	std::cout << "ClockSynchronizerServer stopped\n";
}

// vagy DTime?
long long SF::GetOffsetFromServerTime(std::string address, long long n_msgs, zmq::context_t context) { // address = "tcp://localhost:5555"
																									   //  Prepare socket
	zmq::socket_t socket(context, ZMQ_REQ);
	std::cout << "Connecting to server...";
	socket.connect(address);
	//  Do n_msgs requests, waiting each time for a response
	long long sumoffset = 0;
	std::vector<long long> offsets = std::vector<long long>();
	bool firstrun = true;
	zmq::message_t request(8), reply;
	long long sendtime, recvtime, time, offset;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	for (int request_nbr = 0; request_nbr < n_msgs + 1; request_nbr++) {
		littleEndianSerializer.memcpy(request.data(), "SendTime", 8);
		sendtime = duration_since_epoch(Now()).count();
		socket.send(request);
		//  Get the reply.
		auto res = socket.recv(reply);
		if (res.has_value()) {
			if (!firstrun) {
				recvtime = duration_since_epoch(Now()).count();
				littleEndianSerializer.memcpy(&time, reply.data(), 8);
				offset = (recvtime + sendtime) / 2 - time;
				sumoffset += offset;
				if (request_nbr < 5)
					offsets.push_back(offset);
			}
			else {
				firstrun = false;
				std::cout << " done.\n";
			}
		}
		else {
			std::cout << "ClockSynchronisation: connection lost (timout>2sec). Trying again...\nConnecting to server...";
			request_nbr = 0;
			firstrun = true;
			sumoffset = 0;
		}
		request.rebuild(8);
		reply.rebuild(0);
	}
	return static_cast<long long>(round(float(sumoffset) / float(n_msgs)));
}
