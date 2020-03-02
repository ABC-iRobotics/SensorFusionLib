#pragma once
#include"defs.h"
#include"NetworkConfig.h"
#include<map>
#include<mutex>
#include<memory>

namespace SF {

	/*! \brief Abstract class to run a server that replies timestamps to synchronize clocks
	*
	*
	*/
	class ClockSynchronizerServer {
	public:
		virtual void SetAddress(const std::string& address_) = 0; /*!< Set server address (if the server does not run). E.g.: "tcp://*:5678" */

		virtual void StartServer() = 0; /*!< Start the server */

		virtual void StopServer() = 0; /*!< Stop the server */

		virtual bool IsRunning() const = 0;  /*!< Returns if the server is working */

		typedef std::shared_ptr<ClockSynchronizerServer> ClockSynchronizerServerPtr; /*!< std::shared_ptr for ClockSynchronizerServer */
	};

	ClockSynchronizerServer::ClockSynchronizerServerPtr InitClockSynchronizerServer(const std::string& address);
			/*!< Initializes and start a ClockSyncronizerServer with the given address on ZMQ impl. */

	ClockSynchronizerServer::ClockSynchronizerServerPtr InitClockSynchronizerServer(const NetworkConfig::ConnectionData& config);
			/*!< Initializes and start a ClockSyncronizerServer with the given address on ZMQ impl. */

	/*! \brief Abstract class to run a ClockSynchronizer client that connects to Servers with given addresses
	*
	* 
	*/
	class ClockSyncronizerClient {
	public:
		struct PublisherClockProperties {
			struct Offset {
				Time time0;
				DTime offset0;
				double m;
				Offset(Time time, DTime offset, double m_) : time0(time), offset0(offset), m(m_) {}
				DTime Value(Time time = Now()) {
					return duration_cast(offset0 + (time - time0) * m);
				}
			} offset;
			bool inprogress;
			std::string port;
			PublisherClockProperties(std::string port);
			void Set(PublisherClockProperties::Offset offset_);
		};
	private:
		std::map<std::string, std::shared_ptr<PublisherClockProperties>> clockOffsets; // address (192.168.0.1) without port -> offset properties
		std::mutex mutexForClockOffsets;
		bool isRunning;

	protected:
		virtual PublisherClockProperties::Offset SynchronizeClock(const std::string& clockSyncServerAddress) const = 0; /*!< How to connect to a server - implemented by subclass */

	private:
		void Run(); /*!< core of the separated synchronizer thread */

	public:
		ClockSyncronizerClient(); /*!< Constructor */

		void SynchronizePeriphery(const std::string& address, const std::string& port); /*!< Add Servers to be synchronized to */

		void SynchronizePeriphery(const NetworkConfig::ConnectionData& config);

		bool IsClockSynchronisationInProgress(const std::string& address); /*!< Check if synchronisation for a given device is in progress */

		DTime GetOffset(const std::string& address, Time time = Now()); /*!< Get the computed offset (0) if it is not computed*/

		void PrintStatus(); /*!< Print the actual status of the synchronizer client */
	};

	ClockSyncronizerClient* GetPeripheryClockSynchronizerPtr();
		/*!< Get the pointer of the statically inicialized implemented instance */
}
