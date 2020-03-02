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
		throw std::runtime_error("FATAL ERROR: Unkown endianness (in LittleEndianSerializer::LittleEndianSerializer)");
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
		throw std::runtime_error("FATAL ERROR: address of ZMQClockSynchronizerServer cannot be modified during run (in ZMQClockSynchronizerServer::SetAddress).");
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
		throw std::runtime_error("FATAL ERROR: address of ZMQClockSynchronizerServer is an empty string (in ZMQClockSynchronizerServer::Run)");
	std::cout << "ZMQClockSynchronizerServer started\n";
	//  Prepare our context and socket
	zmq::context_t context = zmq::context_t(1);
	zmq::socket_t socket = zmq::socket_t(context, ZMQ_REP);
	try {
		socket.bind(address);
	}
	catch (...) {
		std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to bind to '" + address + "' (in ZMQClockSynchronizerServer::Run)"));
	}
	zmq::message_t reply(8);
	zmq::message_t request;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	while (running) {
		//  Wait for next request from client with timeouts 200ms
		auto res = socket.recv(request, zmq::recv_flags::dontwait);
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
	try {
		socket.connect(address);
	}
	catch (...) {
		std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to connect to '" + address + "' (in DetermineClockOffsetFromZMQServer)"));
	}
	//  Do n_msgs requests, waiting each time for a response
	long long sumoffset = 0;
	std::vector<long long> offsets = std::vector<long long>();
	bool firstrun = true;
	zmq::message_t request(8), reply;
	long long sendtime, recvtime, time, offset_;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	// try to synchronize clocks
	for (int request_nbr = 0; request_nbr < n_msgs + 1; request_nbr++) {
		littleEndianSerializer.memcpy(request.data(), "SendTime", 8);
		sendtime = GetSystemClockTimeInUS();
		auto res1 = socket.send(request, zmq::send_flags::none);
		if (res1.has_value()) {
			//  Get the reply.
			while (!socket.recv(reply).has_value()) {
				std::cout << "ClockSynchronisation: connection lost (timout>2sec). Trying again...\nConnecting to server...";
				request_nbr = 0;
				firstrun = true;
				sumoffset = 0;
			}
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
				std::cout << " first msg: ok.\n";
			}
			
			request.rebuild(8);
			reply.rebuild(0);
		}
		else {
			std::cout << "ClockSynchronisation: connection lost (timout>2sec). Trying again...\nConnecting to server...";
			request_nbr = 0;
			firstrun = true;
			sumoffset = 0;
		}
	}
	std::cout << "Offset determined: " + std::to_string((sumoffset * 1000) / n_msgs) + " ns\n";
	return std::chrono::nanoseconds((sumoffset * 1000) / n_msgs);
}

struct ComputeOffset {
	Time tStart;
	long n;
	long long sumTUS;
	long long sumOffsetUS;
	long long sumTTUS;
	long long sumTOffsetUS;
	ComputeOffset() : n(0), tStart(Now()), sumTUS(0), sumOffsetUS(0), sumTTUS(0), sumTOffsetUS(0) {}
	void Add(Time t_, DTime offset_) {
		long long t = duration_cast(t_ - tStart).count();
		long long offset = offset_.count();
		sumTUS += t;
		sumOffsetUS += offset;
		sumTOffsetUS += t * offset;
		sumTTUS += t * t;
		n++;
	}
	void Reset() {
		n = 0;
		sumTUS = 0;
		sumOffsetUS = 0;
		sumTTUS = 0;
		sumTOffsetUS = 0;
	}
	ZMQClockSyncronizerClient::PublisherClockProperties::Offset Value() {
		return ZMQClockSyncronizerClient::PublisherClockProperties::Offset(tStart + DTime(sumTUS / n),
			DTime(sumOffsetUS / n), double(sumTOffsetUS - sumTUS * sumOffsetUS / n) / double(sumTTUS - sumTUS * sumTUS / n));
	}
};

ZMQClockSyncronizerClient::PublisherClockProperties::Offset SF::ZMQClockSyncronizerClient::SynchronizeClock(const std::string & clockSyncServerAddress) const {
	zmq::context_t context = zmq::context_t(1);	   //  Prepare socket
	zmq::socket_t socket(context, ZMQ_REQ);
	std::cout << "Connecting to server... (" << clockSyncServerAddress << ")";
	try {
		socket.connect(clockSyncServerAddress);
	}
	catch (...) {
		std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to connect to '" + clockSyncServerAddress + "' (in DetermineClockOffsetFromZMQServer)"));
	}
	//  Do n_msgs requests, waiting each time for a response
	long long n_msgs = 10000;
	bool firstrun = true;
	zmq::message_t request(8), reply;
	Time sendtime, recvtime, time;
	long long temp;
	DTime offset_;
	ComputeOffset computer;
	int rvctime = 2000; // milliseconds
	zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
	// try to synchronize clocks
	for (int request_nbr = 0; request_nbr < n_msgs + 1; request_nbr++) {
		littleEndianSerializer.memcpy(request.data(), "SendTime", 8);
		Time sendtime = Now();
		auto res1 = socket.send(request, zmq::send_flags::none);
		if (res1.has_value()) {
			//  Get the reply.
			while (!socket.recv(reply).has_value()) {
				std::cout << "ClockSynchronisation: connection lost (timout>2sec). Trying again...\nConnecting to server...";
				request_nbr = 0;
				firstrun = true;
				computer.Reset();
			}
			if (!firstrun) {
				Time recvtime = Now();
				littleEndianSerializer.memcpy(&temp, reply.data(), 8);
				time = InitFromDurationSinceEpochInMicroSec(temp);
				Time time_ = sendtime + (recvtime - sendtime) / 2;
				offset_ = duration_cast(time_ - time);	// offset = subsTime - sensortime -> substime = sensorTime + offset
				computer.Add(time_, offset_);
			}
			else {
				firstrun = false;
				std::cout << " first msg: ok.\n";
			}
			request.rebuild(8);
			reply.rebuild(0);
		}
		else {
			std::cout << "ClockSynchronisation: connection lost (timout>2sec). Trying again...\nConnecting to server...";
			request_nbr = 0;
			firstrun = true;
			computer.Reset();
		}
	}
	auto offsetobj = computer.Value();
	std::cout << "Done, now: " << offsetobj.Value().count() << " us (m = " << offsetobj.m << ")\n";
	return computer.Value();
}
