#pragma once

#include<map>
#include<thread>
#include"AppLayer.h"

namespace SF {

	/*! \brief Abstract class for reciever layer of applications
	*
	* Implementations: read from logfile, recieve data from network via zmq
	*
	* Destructor of subclasses must call Stop()
	*/
	class Reciever : public AppLayer {
	private:
		using AppLayer::CallbackSamplingTimeOver; /*!< Callback called in each sampling time - input: time */

		using AppLayer::CallbackGotDataMsg; /*!< Callback called if new DataMsg recieved */

		using AppLayer::CallbackGotString; /*!< Callback called if new string recieved */

		using AppLayer::CallbackMsgQueueEmpty; /*!< Callback called if the DataMsgs in the queue were read */

		bool toStop;

		bool isRunning;

		std::thread t;

		virtual void _Run(DTime Ts) = 0; /*!< Reciever thread must be defined by the subclass */

	public:
		enum MsgType { DATAMSG, TEXT, NOTHING };

		static const DTime tWaitNextMsg;

		Reciever(); /*!< Constructor */

		void Start(DTime Ts); /*!< Starts a reciever thread with given sampling time*/

		void Stop(bool waitin = true); /*!< Stops the reciever thread*/

		bool MustStop() const; /*!< Getter to check if the _Run() thread should stop */

		typedef std::shared_ptr<Reciever> RecieverPtr;  /*!< std::shared_ptr to Reciever class*/

		bool IsRunning() const; /*!< To check if the reciever thread is still running */
	};


	/*! \brief Class for output layer of applications
	*
	* By default it only prints messages and DataMsg-s, implementations: log via spd, zmq publisher
	*/
	class Sender : public AppLayer {
		using AppLayer::AddNextLayer;

	public:
		virtual void CallbackSamplingTimeOver(const Time& currentTime = Now()) override; /*!< Callback called in each sampling time - input: time */

		virtual void CallbackGotDataMsg(const DataMsg& msg, const Time& currentTime = Now()) override; /*!< Callback called if new DataMsg recieved */

		virtual void CallbackGotString(const std::string& msg, const Time& currentTime = Now()) override; /*!< Callback called if new string recieved */

		virtual void CallbackMsgQueueEmpty(const Time& currentTime = Now()) override; /*!< Callback called if the DataMsgs in the queue were read */

		typedef std::shared_ptr<Sender> SenderPtr; /*!< std::shared_ptr to Sender class*/
	};
}
