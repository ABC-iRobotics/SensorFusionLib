#pragma once

#include<map>
#include<mutex>
#include"DataMsg.h"

namespace SF {

	/*! \brief Class for output layer of applications
	*
	* By default it only prints messages and DataMsg-s, implementations: log via spd, zmq publisher
	*/
	class Sender {
	public:
		virtual void SendDataMsg(const DataMsg& data); /*!< Send DataMsg via the output layer */

		virtual void SendString(const std::string& str); /*!< Send string via the output layer */

		typedef std::shared_ptr<Sender> SenderPtr; /*!< std::shared_ptr to Sender class*/
	};

	/*! \brief Class for processor layer of applications
	*
	* By default it only forwards messages and DataMsg-s, implementations: filters
	*/
	class Processor {
		std::mutex senderGuard;   /*!< Mutex to guard sender variable */

	protected:
		Sender::SenderPtr sender; /*!< Pointer to the next (Sender) layer */
	public:
		Processor(); /*!< Constructor */

		void SetSender(Sender::SenderPtr sender_); /*!< Set sender the resulting data to be forwarded */

		typedef std::shared_ptr<Processor> ProcessorPtr; /*!< std::shared_ptr to Processor class*/

		virtual void CallbackSamplingTimeOver(const Time& currentTime = Now()); /*!< Callback called in each sampling time - input: time for the filter*/

		virtual void CallbackGotDataMsg(DataMsg& msg, const Time& currentTime = Now());  /*!< Callback called if new DataMsg recieved */

		virtual void CallbackMsgQueueEmpty(const Time& currentTime = Now());  /*!< Callback called if the DataMsgs in the queue were read */
	};

	/*! \brief Abstract class for reciever layer of applications
	*
	* Implementations: read from logfile, recieve data from network via zmq
	*/
	class Reciever {
		std::mutex processorGuard;  /*!< Mutex to guard processor variable */

		Processor::ProcessorPtr processor; /*!< Pointer to the next (Processor) layer */

	protected:
		void CallbackSamplingTimeOver(const Time& currentTime = Now()); /*!< Calls the appropriate callback of the processor if it was set */

		void CallbackGotDataMsg(DataMsg& msg, const Time& currentTime = Now()); /*!< Calls the appropriate callback of the processor if it was set */

		void CallbackMsgQueueEmpty(const Time& currentTime = Now()); /*!< Calls the appropriate callback of the processor if it was set */

	public:
		Reciever(); /*!< Constructor */

		virtual void Start(DTime Ts) = 0;  /*!< Starts a reciever thread with given sampling time*/

		virtual void Stop(bool waitin = true) = 0;   /*!< Stops the reciever thread*/

		virtual void ConnectToAddress(const std::string& address) = 0; /*!< Set addresses of remote sources for network communication */

		void SetProcessor(Processor::ProcessorPtr processor_); /*!< Set Processor::ProcessorPtr to be called with data and event */

		typedef std::shared_ptr<Reciever> RecieverPtr;  /*!< std::shared_ptr to Reciever class*/
	};

	/*! \brief Class for applications constructed by reciever-processor-sender layers
	*
	* Its subclasses are the Logger, Periphery, Filter, Emulator etc...
	*/
	class Application {
		Reciever::RecieverPtr reciever; /*!< Pointer to the Reciever layer */

		Sender::SenderPtr sender; /*!< Pointer to the Sender layer */

		Processor::ProcessorPtr processor; /*!< Pointer to the Processor layer */

	protected:
		Reciever::RecieverPtr GetRecieverPtr() const; /*!< Returns pointer to the reciever layer */

		Sender::SenderPtr GetSenderPtr() const; /*!< Returns pointer to the sender layer */

		Processor::ProcessorPtr GetProcessorPtr() const; /*!< Returns pointer to the processor layer */

	public:
		Application(Reciever::RecieverPtr reciever_,
			Processor::ProcessorPtr processor_, Sender::SenderPtr sender_); /*!< Constructor */

		void Start(DTime Ts); /*!< Start recieving and corresponding events in a separated thread */

		void Stop(); /*!< Stop the separated thread */

		typedef std::shared_ptr<Application> ApplicationPtr; /*!< std::shared_ptr to Application class*/
	};
}
