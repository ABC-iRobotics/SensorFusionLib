#pragma once
#include <zmq.hpp>
#include "Reciever.h"
#include "NetworkConfig.h"

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
		/*! \brief Struct that describes data necessary to connect to peripheries
		*
		* The address of the Periphery must be specified and if it is subscribed to string msgs too.
		*
		* Subscriptions can be specified to SF::OperationType, ID and SF::DataType - see the defined constructors.
		*/
		struct PeripheryProperties {
			OperationType source; /*!< Subscription: OperationType if it was specified (as SENSOR/...) */
			unsigned char ID; /*!< Subscription: ID if it was specified */
			DataType type; /*!< Subscription: DataType if it was specified (as STATE/OUTPUT/...)*/
			std::string address; /*!< Address of the periphery */
			bool getstrings; /*!< If the string msgs must be recieved too */
			unsigned char nparam; /*!< how many parameters are checked in the datamsg topic - set by the constructors */
			unsigned long long nRecieved = 0; /*!< Number of recieved msgs */
			PeripheryProperties() = delete;
			PeripheryProperties(const std::string& address_,
				bool getstrings_ = false); /*!< To subscribe to the address to recieve arbitrary datamsgs */
			PeripheryProperties(OperationType source_, const std::string& address_,
				bool getstrings_ = false); /*!< To subscribe to the address to recieve datamsgs with given OperationType */
			PeripheryProperties(OperationType source_, unsigned char ID_,
				const std::string& address_, bool getstrings_ = false); /*!< To subscribe to the address to recieve datamsgs with given OperationType and ID */
			PeripheryProperties(OperationType source_, unsigned char ID_,
				DataType type_, const std::string& address_, bool getstrings_ = false); /*!< To subscribe to the address to recieve datamsgs with given OperationType, ID and DataType */
		};

		ZMQReciever(std::vector<PeripheryProperties> periferies
			= std::vector<PeripheryProperties>()); /*!< Constructor */

		~ZMQReciever();

		void AddPeriphery(const PeripheryProperties& prop); /*!< Add peripheries for networked recievers */

		void AddPeripheries(const NetworkConfig& config);

		unsigned long long GetNumOfRecievedMsgs(int n); /*!< Get number of recieved msgs of the i-th periphery*/

		void Pause(bool pause_); /*!< Pause the recieving and processing thread */

	private:
		bool pause;

		std::vector<PeripheryProperties> peripheryProperties;
		std::mutex peripheryPropertiesMutex;

		struct SocketHandler {
			SocketHandler(const PeripheryProperties& prop, zmq::context_t& context);
			std::shared_ptr<zmq::socket_t> socket;
		};
		
		MsgType _ProcessMsg(zmq::message_t& topic, zmq::message_t& msg, const std::string& address); // returns if got DataMsg

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

		void CallbackGotDataMsg(const DataMsg& msg, const Time& currentTime = Now()) override; /*!< Callback called if new DataMsg recieved */

		void CallbackGotString(const std::string& msg, const Time& currentTime = Now()) override; /*!< Callback called if new string recieved */
	};
}
