#include "ZMQCommunication.h"
#include"msgcontent2buf.h"

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
	socket->connect(prop.address.c_str());
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

unsigned long long SF::ZMQReciever::GetNumOfRecievedMsgs(int n) {
	peripheryPropertiesMutex.lock();
	long long out = peripheryProperties[n].nRecieved;
	peripheryPropertiesMutex.unlock();
	return out;
}

void SF::ZMQReciever::Pause(bool pause_) {
	pause = pause_;
}

void SF::ZMQReciever::_ProcessMsg(zmq::message_t & topic, zmq::message_t & msg) {
	char* t = static_cast<char*>(topic.data());
	switch (t[0]) {
	case 'd': {
		if (topic.size() != 4)
			perror("ZMQCommunication::_ProcessMsg corrupted datamsg topic - d.");
		unsigned char ID = static_cast<unsigned char>(t[2]);
		OperationType source = static_cast<OperationType>(t[1]);
		DataType type = static_cast<DataType>(t[3]);
		if (VerifyDataMsgContent(msg.data(), (int)msg.size()))
			CallbackGotDataMsg(InitDataMsg(msg.data(), source, ID, type));
		else
			perror("ZMQCommunication::_ProcessMsg corrupted datamsg buffer.");
		return;
	}
	case 'i':
		if (topic.size() != 1)
			perror("ZMQCommunication::_ProcessMsg corrupted string topic");
		CallbackGotString(std::string(static_cast<char*>(msg.data()), msg.size()));
		return;
	default:
		perror("ZMQCommunication::_ProcessMsg corrupted topic.");
	}
}

bool SF::ZMQReciever::_PollItems(zmq::pollitem_t * items, int nItems, int TwaitMilliSeconds, std::vector<SocketHandler>& socketProperties) {
	zmq::poll(&items[0], nItems, TwaitMilliSeconds);
	bool out = false;
	for (int i = 0; i < nItems; i++)
		if (items[i].revents & ZMQ_POLLIN) {
			auto socket = socketProperties[i].socket;
			zmq::multipart_t t;
			if (t.recv(*socket, ZMQ_DONTWAIT)) {
				peripheryPropertiesMutex.lock();
				peripheryProperties[i].nRecieved++;
				peripheryPropertiesMutex.unlock();
				_ProcessMsg(t[0], t[1]);
				out = true;
			}
		}
	return out;
}

SF::ZMQSender::~ZMQSender() {
	zmq_socket.close();
	zmq_context.close();
}

SF::ZMQSender::ZMQSender(const std::string & address, int hwm) : zmq_context(2) {
	zmq_socket = zmq::socket_t(zmq_context, ZMQ_PUB);
	zmq_socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
	zmq_socket.bind(address);
}

void SF::ZMQSender::SendDataMsg(const DataMsg & data) {
	zmq::multipart_t msg;
	unsigned char topicbuf[4];
	topicbuf[0] = 'd';
	topicbuf[1] = to_underlying<OperationType>(data.GetDataSourceType());
	topicbuf[2] = data.GetSourceID();
	topicbuf[3] = to_underlying<DataType>(data.GetDataType());
	msg.addmem(topicbuf, 4);
	void* buf;
	int bufsize;
	SerializeDataMsg(data, buf, bufsize);
	msg.addmem(buf, bufsize);
	try {
		msg.send(zmq_socket, ZMQ_DONTWAIT);
	}
	catch (zmq::error_t &e) {
		std::cout << e.what() << std::endl;
	}
	delete buf;
}

void SF::ZMQSender::SendString(const std::string & data) {
	zmq::multipart_t msg;
	char i = 'i';
	msg.addmem(&i, 1);
	msg.addmem(&data.c_str()[0], data.length());
	try {
		msg.send(zmq_socket, ZMQ_DONTWAIT);
	}
	catch (zmq::error_t &e) {
		std::cout << e.what() << std::endl;
	}
}
