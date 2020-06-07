#pragma once
#include <thread>
#include <mutex>
#include "NetworkConfig.h"
#include "comm_defs.h"
#include "DataMsg.h"
#include <zmq.hpp>

namespace SF {

	class ZMQReciever {
		bool toStop;

		bool isRunning;

		std::thread t;

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

		unsigned long long GetNumOfRecievedMsgs(int n); /*!< Get number of recieved msgs of the n-th periphery*/

		void Pause(bool pause_); /*!< Pause the recieving and processing thread */

		void Start(DTime Ts); /*!< Starts a reciever thread with given sampling time*/

		void Stop(bool waitin = true); /*!< Stops the reciever thread*/

		bool MustStop() const; /*!< Getter to check if the _Run() thread should stop */

		bool IsRunning() const; /*!< To check if the reciever thread is still running */

	protected:
		virtual void SamplingTimeOver(const Time& currentTime) = 0; /*!< Must called in each sampling time - input: time */

		virtual void SaveDataMsg(const DataMsg& msg, const Time& currentTime) = 0; /*!< Must called if new DataMsg recieved */

		virtual void MsgQueueEmpty(const Time& currentTime) = 0; /*!< Must called if the DataMsgs in the queue were read */

		virtual void SaveString(const std::string& msg, const Time& currentTime) = 0;

	private:
		bool pause;

		std::vector<PeripheryProperties> peripheryProperties;
		std::mutex peripheryPropertiesMutex;

		struct SocketHandler {
			SocketHandler(const PeripheryProperties& prop, zmq::context_t& context);
			std::shared_ptr<zmq::socket_t> socket;
		};

		MsgType _ProcessMsg(zmq::message_t& topic, zmq::message_t& msg, const std::string& address); // returns if got DataMsg

		void _Run(DTime Ts);
	};
}
