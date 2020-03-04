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

	std::cout << "ZMQClockSynchronizerServer stopped\n";
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
}

Offset::OffsetMeasResult ZMQClockSyncronizerClient::DetermineOffset(const std::string& address, long long n) {
	if (sockets.find(address) == sockets.end()) {
		std::shared_ptr<zmq::socket_t> socket = std::make_shared<zmq::socket_t>(context, ZMQ_REQ);
		socket->setsockopt(ZMQ_LINGER, 0); // send the msg or not, but waiting for connection is a waste of time....
		socket->setsockopt(ZMQ_RCVTIMEO, 1000);
		socket->setsockopt(ZMQ_SNDTIMEO, 500);
		try {
			std::cout << "Connecting to server... (" << address << ")";
			socket->connect(address);
		}
		catch (...) {
			std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to connect to '"
				+ address + "' (in DetermineClockOffsetFromZMQServer)"));
		}
		sockets.insert(std::pair<std::string, std::shared_ptr<zmq::socket_t>>(address, socket));
	}
	zmq::socket_t& socket = *sockets.find(address)->second;

	zmq::pollitem_t item = { socket, 0, ZMQ_POLLIN, 0 };

	//  Do n_msgs requests, waiting each time for a response
	bool firstrun = true;
	zmq::message_t request(8), reply;
	Time sendtime, recvtime, time;
	Offset::OffsetMeasResult out;
	DTime out_dt;
	long long temp;

	// try to synchronize clocks
	for (int request_nbr = 0; request_nbr < n + 1; request_nbr++) {
		littleEndianSerializer.memcpy(request.data(), "SendTime", 8);
		Time sendtime = Now();
		try {
			if (!socket.send(request, zmq::send_flags::none).has_value())
				throw ClockSyncConnectionError(address);
		}
		catch (...) {
			socket.recv(reply);
			throw ClockSyncConnectionError(address);
		}

		zmq::poll(&item, 1, 2000);
		
		if (!(item.revents & ZMQ_POLLIN))
			throw ClockSyncConnectionError(address);

		//  Get the reply.
		if (!socket.recv(reply).has_value())
			throw ClockSyncConnectionError(address);
		Time recvtime = Now();
		littleEndianSerializer.memcpy(&temp, reply.data(), 8);
		time = InitFromDurationSinceEpochInMicroSec(temp);
		Time time_ = sendtime + (recvtime - sendtime) / 2;
		DTime dt = duration_cast(recvtime - sendtime);
		DTime offset_ = duration_cast(time_ - time);	// offset = subsTime - sensortime -> substime = sensorTime + offset
		if (firstrun || dt < out_dt) {
			firstrun = false;
			out_dt = dt;
			out = Offset::OffsetMeasResult(time_, offset_);
		}
		request.rebuild(8);
		reply.rebuild(0);
	}
	//std::cout << "Done; now: " << out.offset.count() << "\n";
	return out;
}

ClockSyncronizerClient* SF::GetPeripheryClockSynchronizerPtr() {
	static ZMQClockSyncronizerClient zmqClockSyncronizerClient;
	return &zmqClockSyncronizerClient;
}
