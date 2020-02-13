#include "RecieverViaZMQ.h"
#include "IClockSynchronizer.h"
#include "msg2buf.h"

using namespace SF;

void SF::RecieverViaZMQ::Run(DTime Ts) {
	int sg = 0;
	socket.setsockopt(ZMQ_CONFLATE, sg);

	zmq::message_t data;
	DataMsg dataMsg;

	Time start = Now();
	long iIteration = 1;
	while (!MustStop()) {
		Time end = start + iIteration * Ts;
		while (Now() < end) {
			bool recv = RecieveDataMsg(dataMsg, duration_cast(end-Now()));
			while (recv) {
				// Apply offset
				if (!GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(dataMsg.GetSourceID())) {
					dataMsg.ApplyOffset(GetPeripheryClockSynchronizerPtr()->GetOffset(dataMsg.GetSourceID()));
					CallbackGotDataMsg(dataMsg);
				}
				recv = RecieveDataMsg(dataMsg, DTime(0));
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

bool SF::RecieverViaZMQ::RecieveDataMsg(DataMsg & dataMsg, DTime Twait) {
	if (Twait < DTime(0))
		return false;
	while (!MustStop()) {

		socketguard.lock();
		zmq::detail::recv_result_t res;
		if (Twait == DTime(0))
			res = socket.recv(data, zmq::recv_flags::dontwait);
		else {
			rvctime = std::chrono::duration_cast<std::chrono::milliseconds>(Twait).count();
			socket.setsockopt(ZMQ_RCVTIMEO, rvctime);
			res = socket.recv(data);
		}
		socketguard.unlock();

		if (!res.has_value())
			return false;

		bool isDataMsg = ExtractBufIf(data.data(), data.size(), dataMsg);
		if (isDataMsg)
			return true;
	}
	return false;
}
