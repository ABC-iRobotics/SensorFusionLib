#pragma once
#include <thread>
#include "ClockSynchronizer.h"
#include "zmq.hpp"

namespace SF {

	/*! \brief ZMQ based implementation of ClockSynchronizerServer
	* 
	* A request-reply server, that answers its SystemClock in us.
	*/
	class ZMQClockSynchronizerServer : public ClockSynchronizerServer {
		std::string address;
		std::thread serverthread;
		bool running;
	public: 
		ZMQClockSynchronizerServer(const std::string& address_=""); /*!< Constructor */

		void SetAddress(const std::string& address_) override;

		void StartServer() override;

		void StopServer() override;

		bool IsRunning() const override;

		~ZMQClockSynchronizerServer(); /*!< Destructor */

	private:
		void Run();
	};

	/*! \brief ZMQ based implementation of ClockSyncronizerClient
	*
	* It implements the private method of connecting to a server and computing th offset
	*
	* Use function
	* ClockSyncronizerClient* GetPeripheryClockSynchronizerPtr();
	* to get the pointer to the static client, and use functions
	* void ClockSyncronizerClient::SynchronizePeriphery(unsigned char ID, const std::string& address);
	* bool ClockSyncronizerClient::IsClockSynchronisationInProgress(unsigned char ID);
	* DTime ClockSyncronizerClient::GetOffset(unsigned char ID);
	* void ClockSyncronizerClient::PrintStatus();
	* for its functionalities
	*/
	class ZMQClockSyncronizerClient : public ClockSyncronizerClient {
		zmq::context_t context;

		std::map<std::string, std::shared_ptr<zmq::socket_t>> sockets;

		Offset::OffsetMeasResult DetermineOffset(const std::string& address, long long n) override;

	public:
		ZMQClockSyncronizerClient() : context(1), sockets(std::map<std::string, std::shared_ptr<zmq::socket_t>>()) {}
	};
}
