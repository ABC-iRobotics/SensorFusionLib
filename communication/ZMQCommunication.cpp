#include "ZMQCommunication.h"
#include"msgcontent2buf.h"
#include"ClockSynchronizer.h"
#include"zmq_addon.hpp"

using namespace SF;

SF::ZMQReciever::PeripheryProperties::PeripheryProperties(const std::string& address_, bool getinfos_) :
	address(address_), getstrings(getinfos_), nparam(0) {}
SF::ZMQReciever::PeripheryProperties::PeripheryProperties(OperationType source_, const std::string& address_, bool getinfos_) : source(source_),
	address(address_), getstrings(getinfos_), nparam(1) {}
SF::ZMQReciever::PeripheryProperties::PeripheryProperties(OperationType source_, unsigned char ID_,
	const std::string& address_, bool getinfos_) : source(source_),
	ID(ID_), address(address_), getstrings(getinfos_), nparam(2) {}  // To add an address and recieve datamsgs with given type and ID
SF::ZMQReciever::PeripheryProperties::PeripheryProperties(OperationType source_,
	unsigned char ID_, DataType type_, const std::string& address_, bool getinfos_) :
	source(source_), ID(ID_), address(address_), getstrings(getinfos_), type(type_), nparam(3) {}

SF::ZMQReciever::SocketHandler::SocketHandler(const PeripheryProperties& prop, zmq::context_t& context) :
	socket(std::make_shared<zmq::socket_t>(context, ZMQ_SUB)) {
	char topic[4];
	topic[0] = 'd';
	topic[1] = to_underlying<OperationType>(prop.source);
	topic[2] = prop.ID;
	topic[3] = to_underlying<DataType>(prop.type);
	socket->setsockopt(ZMQ_SUBSCRIBE, &topic[0], prop.nparam + 1);
	if (prop.getstrings)
		socket->setsockopt(ZMQ_SUBSCRIBE, "i", 1);
	try {
		socket->connect(prop.address.c_str());
	}
	catch (zmq::error_t& t) {
		socket->close();
		std::cout << "ZMQ ERROR: connecting to address '" << prop.address << "' (" <<   t.what() << ")" << std::endl;
		//std::throw_with_nested(std::runtime_error("sg"));
		//throw std::runtime_error("-");// std::runtime_error("FATAL ERROR: ZMQ unable to connect to '" + prop.address + "' (in ZMQReciever::SocketHandler::SocketHandler)");
		exit(EXIT_FAILURE); // I cannot rethrow i dont know why...
	}
}

SF::ZMQReciever::~ZMQReciever() {
	Stop();
}

SF::ZMQReciever::ZMQReciever(std::vector<PeripheryProperties> periferies)
	: peripheryProperties(periferies), pause(false) {}

void SF::ZMQReciever::AddPeriphery(const PeripheryProperties & prop) {
	peripheryPropertiesMutex.lock();
	peripheryProperties.push_back(prop);
	peripheryPropertiesMutex.unlock();
}

void SF::ZMQReciever::AddPeripheries(const NetworkConfig & config) {
	// Add clocks to synchronise
	for (auto clock : config.clockSyncData)
		GetPeripheryClockSynchronizerPtr()->SynchronizePeriphery(clock.second);
	// Add peripheries
	for (auto periphery : config.peripheryData)
		AddPeriphery(PeripheryProperties(OperationType::SENSOR, periphery.second.RecieverAddress(), true));
}

unsigned long long SF::ZMQReciever::GetNumOfRecievedMsgs(int n) {
	peripheryPropertiesMutex.lock();
	long long out = peripheryProperties[n].nRecieved;
	peripheryPropertiesMutex.unlock();
	return out;
}

void SF::ZMQReciever::Pause(bool pause_) {
	pause = pause_;
}

SF::ZMQReciever::MsgType SF::ZMQReciever::_ProcessMsg(zmq::message_t & topic, zmq::message_t & msg, const std::string& address) {
	char* t = static_cast<char*>(topic.data());
	switch (t[0]) {
	case 'd': {
		if (topic.size() != 4)
			throw std::runtime_error("FATAL ERROR: corrupted datamsg topic got (in ZMQCommunication::_ProcessMsg)");
		unsigned char ID = static_cast<unsigned char>(t[2]);
		// Apply offset
		OperationType source = static_cast<OperationType>(t[1]);
		DataType type = static_cast<DataType>(t[3]);
		if (VerifyDataMsgContent(msg.data(), (int)msg.size())) {
			if (!GetPeripheryClockSynchronizerPtr()->IsClockSynchronisationInProgress(address))
				ForwardDataMsg(InitDataMsg(msg.data(), source, ID, type, GetPeripheryClockSynchronizerPtr()->GetOffset(address)));
		}
		else
			throw std::runtime_error("FATAL ERROR: corrupted datamsg buffer got (in ZMQCommunication::_ProcessMsg)");
		return MsgType::DATAMSG;
	}
	case 'i':
		if (topic.size() != 1)
			throw std::runtime_error("FATAL ERROR: corrupted string topic got (in ZMQCommunication::_ProcessMsg)");
		ForwardString(std::string(static_cast<char*>(msg.data()), msg.size()));
		return MsgType::TEXT;
	default:
		throw std::runtime_error("FATAL ERROR: corrupted topic got (in ZMQCommunication::_ProcessMsg)");
	}
}

void SF::ZMQReciever::_Run(DTime Ts) {
	zmq::context_t context(2);
	std::vector<SocketHandler> socketProperties = std::vector<SocketHandler>();
	zmq::pollitem_t items[100];
	Time tLast;
	int nItems = 0;
	int msToEllapse;
	// For managing sampling time
	Time tNext = Now() + Ts;
	// Main iteration
	bool gotDataMsg = false;
	while (!MustStop()) {
		// Create sockets if there are elements of socketproperties not set
		peripheryPropertiesMutex.lock();
		for (size_t i = socketProperties.size(); i < peripheryProperties.size(); i++) {
			socketProperties.push_back(SocketHandler(peripheryProperties[i], context));
			if (nItems == 100)
				throw std::runtime_error("FATAL ERROR: too many sockets to be polled (>100) (in ZMQReciever::_Run) increase nItems_max!");
			else {
				items[nItems] = { *(socketProperties[i].socket), 0, ZMQ_POLLIN, 0 };
				nItems++;
			}
		}
		peripheryPropertiesMutex.unlock();
		// wait if pause
		while (pause)
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// handle step
		if (Now() > tNext) {
			ForwardSamplingTimeOver();
			tNext += Ts;
			gotDataMsg = false;
			continue;
		}
		// poll: time to compute it
		if (gotDataMsg)
			msToEllapse = 0;
		else {
			msToEllapse = static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(tNext - Now()).count());
			msToEllapse = msToEllapse < 0 ? 0 : msToEllapse;
		}
		// poll
		zmq::poll(&items[0], nItems, msToEllapse);
		bool gotSg = false;
		bool gotDataMsgNow = false;
		for (int i = 0; i < nItems; i++)
			if (items[i].revents & ZMQ_POLLIN) {
				auto socket = socketProperties[i].socket;
				zmq::multipart_t t;
				if (t.recv(*socket, ZMQ_DONTWAIT)) {
					peripheryPropertiesMutex.lock();
					peripheryProperties[i].nRecieved++;
					peripheryPropertiesMutex.unlock();
					MsgType type = _ProcessMsg(t[0], t[1], peripheryProperties[i].address);
					gotSg |= type != MsgType::NOTHING;
					gotDataMsgNow |= type == MsgType::DATAMSG;
				}
			}
		gotDataMsg |= gotDataMsgNow;
		if (gotDataMsg && !gotSg && Now() > tLast + tWaitNextMsg) {
			ForwardMsgQueueEmpty(); // if there were msg read
			gotDataMsg = false;
		}
		if (gotDataMsgNow)
			tLast = Now();
	}
}

SF::ZMQSender::~ZMQSender() {
	zmq_socket.close();
	zmq_context.close();
}

SF::ZMQSender::ZMQSender(const std::string & address, int hwm) : zmq_context(2) {
	zmq_socket = zmq::socket_t(zmq_context, ZMQ_PUB);
	zmq_socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
	try {
		zmq_socket.bind(address);
	}
	catch (zmq::error_t t) {
		zmq_socket.close();
		zmq_context.close();
		std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to bind to '" + address + "' (in ZMQSender::ZMQSender) (" + t.what() + ")"));
	}
}

void SF::ZMQSender::CallbackGotDataMsg(const DataMsg & data, const Time& currentTime) {
	zmq::multipart_t msg;
	unsigned char topicbuf[4];
	topicbuf[0] = 'd';
	topicbuf[1] = to_underlying<OperationType>(data.GetDataSourceType());
	topicbuf[2] = data.GetSourceID();
	topicbuf[3] = to_underlying<DataType>(data.GetDataType());
	msg.addmem(topicbuf, 4);
	unsigned char* buf;
	int bufsize;
	SerializeDataMsg(data, buf, bufsize);
	msg.addmem((void*)buf, bufsize);
	try {
		msg.send(zmq_socket, ZMQ_DONTWAIT);
	}
	catch (zmq::error_t &e) {
		std::throw_with_nested(std::runtime_error("FATAL ERROR: sending ZMQ msg (MQSender::CallbackGotDataMsg)"));
	}
	delete buf;
}

void SF::ZMQSender::CallbackGotString(const std::string & data, const Time& currentTime) {
	zmq::multipart_t msg;
	char i = 'i';
	msg.addmem(&i, 1);
	msg.addmem(&data.c_str()[0], data.length());
	try {
		msg.send(zmq_socket, ZMQ_DONTWAIT);
	}
	catch (zmq::error_t &e) {
		std::throw_with_nested(std::runtime_error("FATAL ERROR: sending ZMQ msg (MQSender::CallbackGotString)"));
	}
}
