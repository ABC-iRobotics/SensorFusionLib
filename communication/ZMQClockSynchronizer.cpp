#include "ZMQClockSynchronizer.h"
#include <iostream>
#include <cstdint>
#include <cstdio>
#include <zmq.hpp>

using namespace SF;

/*! \brief Serializer for integers to memcpy from/to LittleEndian convention according to the actual processor
*
*/
class LittleEndianSerializer {
	enum Endianness { LITTLE_ENDIAN_ = 0, BIG_ENDIAN_ } endianness;
public:
	LittleEndianSerializer() /*!< Constructor */
	{
		uint32_t word = 0x0A0B0C0D; // An unsigned 32-bit integer.
		char *pointer = (char *)&word; // A pointer to the first octet of the word.
		if (pointer[0] == 0x0D) {
			endianness = LITTLE_ENDIAN_;
			return;
		}
		if (pointer[0] == 0x0A) {
			endianness = BIG_ENDIAN_;
			return;
		}
		perror("Unkown endianness");
	}

	/*! \brief To copy integer with LITTLE_ENDIAN convention */
	void memcpy(void* dest, const void* src, int size)
	{
		const char *srcptr = (const char *)src;
		char *destptr = (char *)dest;
		if (endianness == LITTLE_ENDIAN_)
			for (int i = 0; i < size; i++)
				destptr[i] = srcptr[i];
		if (endianness == BIG_ENDIAN_)
			for (int i = 0; i < size; i++)
				destptr[size - i - 1] = srcptr[i];
	}
};

static LittleEndianSerializer littleEndianSerializer;

SF::ZMQClockSynchronizerServer::ZMQClockSynchronizerServer(const std::string & address_) :
	running(false), address(address_) {}

void SF::ZMQClockSynchronizerServer::SetAddress(const std::string & address_) {
	if (!running)
		address = address_;
	else
		perror("ZMQClockSynchronizerServer::SetAddress Address cannot be modified during run...");
}

ClockSyncronizerClient* SF::GetPeripheryClockSynchronizerPtr() {
	static ZMQClockSyncronizerClient zmqClockSyncronizerClient;
	return &zmqClockSyncronizerClient;
}

ClockSynchronizerServer::ClockSynchronizerServerPtr SF::InitClockSynchronizerServer(const std::string& address) {
	auto out = std::make_shared<ZMQClockSynchronizerServer>(address);
	out->StartServer();
	return out;
}

void SF::ZMQClockSynchronizerServer::StartServer() {
	if (!running) {
		running = true;
		serverthread = std::thread(&ZMQClockSynchronizerServer::Run, this);
	}
}

void SF::ZMQClockSynchronizerServer::StopServer() {
	if (running) {
		running = false;
		serverthread.join();
	}
}

bool SF::ZMQClockSynchronizerServer::IsRunning() const {
	return running;
}

SF::ZMQClockSynchronizerServer::~ZMQClockSynchronizerServer() {
	StopServer();
}

long long GetSystemClockTimeInUS() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void SF::ZMQClockSynchronizerServer::Run() {
	if (address.compare("") == 0)
		perror("ZMQClockSynchronizerServer::Run(): Address is an empty string!");
	std::cout << "ZMQClockSynchronizerServer started\n";
	//  Prepare our context and socket
	zmq::context_t context = zmq::context_t(1);
	zmq::socket_t socket = zmq::socket_t(context, ZMQ_REP);
	socket.bind(address);
	zmq::message_t reply(8);
	zmq::message_t request;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	while (running) {
		//  Wait for next request from client with timeouts 200ms
		auto res = socket.recv(request);
		if (res.has_value()) { // && res.value() != EAGAIN) {
			long long time = GetSystemClockTimeInUS();
			//  Send reply back to client
			littleEndianSerializer.memcpy(reply.data(), &time, 8);
			socket.send(reply, zmq::send_flags::dontwait);
			reply.rebuild(8);
			request.rebuild();
		}
	}
	std::cout << "ZMQClockSynchronizerServer stopped\n";
}

std::chrono::nanoseconds SF::DetermineClockOffsetFromZMQServer(const std::string& address,
	long long n_msgs, int zmq_io_threads) {
	zmq::context_t context = zmq::context_t(zmq_io_threads);	   //  Prepare socket
	zmq::socket_t socket(context, ZMQ_REQ);
	std::cout << "Connecting to server... (" << address << ")";
	socket.connect(address);
	//  Do n_msgs requests, waiting each time for a response
	long long sumoffset = 0;
	std::vector<long long> offsets = std::vector<long long>();
	bool firstrun = true;
	zmq::message_t request(8), reply;
	long long sendtime, recvtime, time, offset_;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	for (int request_nbr = 0; request_nbr < n_msgs + 1; request_nbr++) {
		littleEndianSerializer.memcpy(request.data(), "SendTime", 8);
		sendtime = GetSystemClockTimeInUS();
		socket.send(request, zmq::send_flags::none);
		//  Get the reply.
		auto res = socket.recv(reply);
		if (res.has_value()) {
			if (!firstrun) {
				recvtime = GetSystemClockTimeInUS();
				littleEndianSerializer.memcpy(&time, reply.data(), 8);
				offset_ = (recvtime + sendtime) / 2 - time;	// offset = subsTime - sensortime -> substime = sensorTime + offset
				sumoffset += offset_;
				if (request_nbr < 5)
					offsets.push_back(offset_);
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
	return std::chrono::nanoseconds((sumoffset * 1000) / n_msgs);
}

DTime SF::ZMQClockSyncronizerClient::SynchronizeClock(const std::string & clockSyncServerAddress) const {
	return duration_cast(DetermineClockOffsetFromZMQServer(clockSyncServerAddress));
}
