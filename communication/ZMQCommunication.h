#pragma once
#include <zmq.hpp>
#include"Application.h"
#include"msgcontent2buf.h"

#include"zmq_addon.hpp"

namespace SF {
	class ZMQRecieve : public Reciever {
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

		ZMQRecieve(std::vector<PeripheryProperties> periferies
			= std::vector<PeripheryProperties>());

		~ZMQRecieve();

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
		
		void ProcessMsg(zmq::message_t& topic, zmq::message_t& msg) {
			char* t = static_cast<char*>(topic.data());
			switch (t[0]) {
			case 'd': {
				if (topic.size() != 4)
					perror("ZMQCommunication::ProcessMsg corrupted datamsg topic - d.");
				unsigned char ID = static_cast<unsigned char>(t[2]);
				OperationType source = static_cast<OperationType>(t[1]);
				DataType type = static_cast<DataType>(t[3]);
				if (VerifyDataMsgContent(msg.data(), (int)msg.size()))
					CallbackGotDataMsg(InitDataMsg(msg.data(), source, ID, type));
				else
					perror("ZMQCommunication::ProcessMsg corrupted datamsg buffer.");
				return;
			}
			case 'i':
				if (topic.size() != 1)
					perror("ZMQCommunication::ProcessMsg corrupted string topic");
				CallbackGotString(std::string(static_cast<char*>(msg.data()), msg.size()));
				return;
			default:
				perror("ZMQCommunication::ProcessMsg corrupted topic.");
			}
		}

		void Run(DTime Ts) override {
			zmq::context_t context(2);
			std::vector<SocketHandler> socketProperties = std::vector<SocketHandler>();
			zmq::pollitem_t items[100];
			int nItems = 0;

			zmq::message_t msg;
			zmq::message_t topic;

			

			Time start;
			while (!MustStop()) {
				// Create sockets if there are elements of socketproperties not set
				peripheryPropertiesMutex.lock();
				for (size_t i = socketProperties.size(); i < peripheryProperties.size(); i++) {
					socketProperties.push_back(SocketHandler(peripheryProperties[i], context));
					if (nItems == 100)
						perror("Error");
					else {
						items[nItems] = { *(socketProperties[i].socket), 0, ZMQ_POLLIN, 0 };
						nItems++;
					}
				}
				peripheryPropertiesMutex.unlock();
				// wait if
				while (pause)
					std::this_thread::sleep_for(std::chrono::milliseconds(100));

				// target objects
				int rc = 0;
				zmq::poll(&items[0], nItems, static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(Ts).count()));

				for (int i = 0; i < nItems; i++)
					if (items[i].revents & ZMQ_POLLIN) {
						auto socket = socketProperties[i].socket;
						zmq::multipart_t t;
						if (t.recv(*socket, ZMQ_DONTWAIT))
						{
							peripheryPropertiesMutex.lock();
							peripheryProperties[i].nRecieved++;
							peripheryPropertiesMutex.unlock();
							ProcessMsg(t[0], t[1]);
						}
					}
			}
		}
	};


	class ZMQSend : public Sender {
		zmq::socket_t zmq_socket;
		zmq::context_t zmq_context;

	public:
		~ZMQSend();

		ZMQSend(const std::string& address, int hwm = 5);

		void SendDataMsg(const DataMsg& data) override {
			zmq::multipart_t msg;
			unsigned char topicbuf[4];
			topicbuf[0] = 'd';
			topicbuf[1] = to_underlying<OperationType>(data.GetDataSourceType());
			topicbuf[2] = data.GetSourceID();
			topicbuf[3] = to_underlying<DataType>(data.GetDataType());
			msg.addmem(topicbuf,4);
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
		
		void SendString(const std::string& data) override {
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
	};
}
