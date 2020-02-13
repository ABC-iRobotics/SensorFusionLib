#include "RecieverViaZMQ.h"
#include "IClockSynchronizer.h"
#include "msg2buf.h"

using namespace SF;

void SF::RecieverViaZMQ::Run(DTime Ts) {
	int sg = 0;
	socket.setsockopt(ZMQ_CONFLATE, sg);

	zmq::message_t data;
	DataMsg dataMsg;
	int rvctime;

	Time start = Now();
	long iIteration = 1;
	while (!MustStop()) {
		Time end = start + iIteration * Ts;
		while (Now() < end) {
			bool recv = false;
			rvctime = (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - Now()).count(); // milliseconds
			zmq_setsockopt(socket, ZMQ_RCVTIMEO, &rvctime, sizeof(rvctime));
			socketguard.lock();
			auto res = socket.recv(data);
			socketguard.unlock();
			while (res.has_value()) {
				recv = true;
				Buffer b = Buffer(static_cast<unsigned char*>(data.data()), data.size());
				dataMsg = b.ExtractDataMsg();
				// Apply offset
				if (!GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(dataMsg.GetSourceID())) {
					dataMsg.ApplyOffset(GetPeripheryClockSynchronizerPtr()->GetOffset(dataMsg.GetSourceID()));
					CallbackGotDataMsg(dataMsg);
				}
				socketguard.lock();
				res = socket.recv(data, zmq::recv_flags::dontwait); // or set rvctime to zero
				socketguard.unlock();
			}
			if (recv)
				CallbackMsgQueueEmpty();
		}
		CallbackSamplingTimeOver();
		iIteration++;
	}

	sg = 1;
	socket.setsockopt(ZMQ_CONFLATE, sg);
}

SF::RecieverViaZMQ::RecieverViaZMQ() : context(1), Reciever() {
	socket = zmq::socket_t(context, ZMQ_SUB);
	socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
	int sg = 1;
	socket.setsockopt(ZMQ_CONFLATE, sg);
}

SF::RecieverViaZMQ::~RecieverViaZMQ() {
	Stop();
}

void SF::RecieverViaZMQ::ConnectToAddress(const std::string & address) {
	socketguard.lock();
	socket.connect(address);
	socketguard.unlock();
}
