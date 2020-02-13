#pragma once
#include <thread>
#include "IClockSynchronizer.h"

namespace SF {

	/*! \brief ZMQ based implementation of IClockSynchronizerServer
	* 
	* 
	*/
	class ZMQClockSynchronizerServer : public IClockSynchronizerServer {
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

	
	std::chrono::nanoseconds DetermineClockOffsetFromZMQServer(const std::string& address, long long n_msgs = 10000, int zmq_io_threads = 1);
		/*!< To connect to a ZMQClockSyncronizerServer and compute the offset */

	/*! \brief ZMQ based implementation of IClockSyncronizerClient
	*
	*
	*/
	class ZMQClockSyncronizerClient : public IClockSyncronizerClient {
		DTime SynchronizeClock(const std::string& clockSyncServerAddress) const override;
	};
}
