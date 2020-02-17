#pragma once
#include <zmq.hpp>
#include"Application.h"

namespace SF {
	class ZMQReciever : public Reciever {
	public:
		struct PeripheryProperties {
			OperationType source;
			unsigned char ID;
			DataType type;
			std::string address;
			bool getstrings;
			unsigned char nparam; // how many parameters are checked in the datamsg topic - set by the constructors
			unsigned long long nRecieved = 0;
			PeripheryProperties() = delete;
			PeripheryProperties(const std::string& address_,
				bool getstrings_ = false); // To add an address and recieve arbitrary datamsgs
			PeripheryProperties(OperationType source_, const std::string& address_,
				bool getstrings_ = false); // To add an address and recieve datamsgs with given type
			PeripheryProperties(OperationType source_, unsigned char ID_,
				const std::string& address_, bool getstrings_ = false);  // To add an address and recieve datamsgs with given type and ID
			PeripheryProperties(OperationType source_, unsigned char ID_,
				DataType type_, const std::string& address_, bool getstrings_ = false);  // To add an address and recieve datamsgs with given types and ID
		};

		ZMQReciever(std::vector<PeripheryProperties> periferies
			= std::vector<PeripheryProperties>());

		~ZMQReciever();

		void AddPeriphery(const PeripheryProperties& prop);

		unsigned long long GetNumOfRecievedMsgs(int n);

		void Pause(bool pause_);

	private:
		bool pause;

		std::vector<PeripheryProperties> peripheryProperties;
		std::mutex peripheryPropertiesMutex;

		struct SocketHandler {
			SocketHandler(const PeripheryProperties& prop, zmq::context_t& context);
			std::shared_ptr<zmq::socket_t> socket;
		};
		
		void _ProcessMsg(zmq::message_t& topic, zmq::message_t& msg);

		bool _PollItems(zmq::pollitem_t* items, int nItems, int TwaitMilliSeconds, std::vector<SocketHandler>& socketProperties);

		void _Run(DTime Ts) override;
	};

	class ZMQSender : public Sender {
		zmq::socket_t zmq_socket;
		zmq::context_t zmq_context;

	public:
		~ZMQSender();

		ZMQSender(const std::string& address, int hwm = 5);

		void SendDataMsg(const DataMsg& data) override;
		
		void SendString(const std::string& data) override;
	};
}
