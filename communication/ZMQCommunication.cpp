#include "ZMQCommunication.h"

using namespace SF;

SF::ZMQRecieve::PeripheryProperties::PeripheryProperties(const std::string& address_, bool getinfos_) :
	address(address_), getstrings(getinfos_), nparam(0) {}
SF::ZMQRecieve::PeripheryProperties::PeripheryProperties(OperationType source_, const std::string& address_, bool getinfos_) : source(source_),
	address(address_), getstrings(getinfos_), nparam(1) {}
SF::ZMQRecieve::PeripheryProperties::PeripheryProperties(OperationType source_, unsigned char ID_,
	const std::string& address_, bool getinfos_) : source(source_),
	ID(ID_), address(address_), getstrings(getinfos_), nparam(2) {}  // To add an address and recieve datamsgs with given type and ID
SF::ZMQRecieve::PeripheryProperties::PeripheryProperties(OperationType source_,
	unsigned char ID_, DataType type_, const std::string& address_, bool getinfos_) :
	source(source_), ID(ID_), address(address_), getstrings(getinfos_), type(type_), nparam(3) {}

SF::ZMQRecieve::SocketHandler::SocketHandler(const PeripheryProperties& prop, zmq::context_t& context) :
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

SF::ZMQRecieve::~ZMQRecieve() {
	Stop();
}

SF::ZMQRecieve::ZMQRecieve(std::vector<PeripheryProperties> periferies)
	: peripheryProperties(periferies), pause(false) {}

void SF::ZMQRecieve::AddPeriphery(const PeripheryProperties & prop) {
	peripheryPropertiesMutex.lock();
	peripheryProperties.push_back(prop);
	peripheryPropertiesMutex.unlock();
}

unsigned long long SF::ZMQRecieve::GetNumOfRecievedMsgs(int n) {
	peripheryPropertiesMutex.lock();
	long long out = peripheryProperties[n].nRecieved;
	peripheryPropertiesMutex.unlock();
	return out;
}

void SF::ZMQRecieve::Pause(bool pause_) {
	pause = pause_;
}

SF::ZMQSend::~ZMQSend() {
	zmq_socket.close();
	zmq_context.close();
}

SF::ZMQSend::ZMQSend(const std::string & address, int hwm) : zmq_context(2) {
	zmq_socket = zmq::socket_t(zmq_context, ZMQ_PUB);
	zmq_socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
	zmq_socket.bind(address);
}
