#pragma once
#include <zmq.hpp>
#include"Application.h"

namespace SF {
	/*! \brief ZMQ-based implementation of Reciever class
	*
	* Peripheries to be connected must be defined in the contructor or by using method AddPeriphery
	*
	* \htmlonly
	* <embed src="Recieving messages.pdf" width="800px" height="550px" href="Recieving messages.pdf"></embed>
	* \endhtmlonly
	*/
	class ZMQReciever : public Reciever {
	public:
		ZMQReciever(std::vector<PeripheryProperties> periferies
			= std::vector<PeripheryProperties>());

		~ZMQReciever();

		void AddPeriphery(const PeripheryProperties& prop) override; /*!< Add peripheries for networked recievers */

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
		
		MsgType _ProcessMsg(zmq::message_t& topic, zmq::message_t& msg); // returns if got DataMsg

		bool _PollItems(zmq::pollitem_t* items, int nItems, int TwaitMilliSeconds, std::vector<SocketHandler>& socketProperties);

		void _Run(DTime Ts) override;
	};

	/*! \brief ZMQ-based implementation of Sender class
	*
	* The class is NOT thread safe! The same thread must initialize the class and call its functions.
	*/
	class ZMQSender : public Sender {
		zmq::socket_t zmq_socket;
		zmq::context_t zmq_context;

	public:
		~ZMQSender(); /*!< Destructor */

		ZMQSender(const std::string& address, int hwm = 5); /*!< Constructor that initializes the zmq::context and the ZMQ_PUB socket, with out queue size "hwm" */

		void SendDataMsg(const DataMsg& data) override; /*!< Sending DataMsg */
		
		void SendString(const std::string& data) override; /*!< Sending string */
	};
}
