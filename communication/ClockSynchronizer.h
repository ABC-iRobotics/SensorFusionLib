#pragma once
#include"defs.h"
#include"NetworkConfig.h"
#include<map>
#include<vector>
#include<mutex>
#include<memory>
#include<thread>
#include<iostream>

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


	class Offset {
	public:
		struct OffsetMeasResult {
			Time t;
			DTime offset;
			OffsetMeasResult(Time t_, DTime offset_);
			OffsetMeasResult() {};
		};

		Offset() {}

		Offset(OffsetMeasResult res1, OffsetMeasResult res2);

		DTime GetOffset(Time t);

		void Add(OffsetMeasResult value);

		bool IsInitialized() const;
	
	private:
		std::vector<OffsetMeasResult> values;
		Time t0;
		DTime offset0;
		double m;
		bool messy;

		void Update();
	};

	class ClockSyncConnectionError : public std::exception {
	private:
		std::string msg;
	public:
		ClockSyncConnectionError(std::string address);

		virtual const char* what() const throw() override;
	};

	/*! \brief Abstract class to run a ClockSynchronizer client that connects to Servers with given addresses
	*
	*
	*/
	class ClockSyncronizerClient {
		class ClockServerProperties {
			Offset offset;
			std::string port;
			std::mutex guard;
		public:
			ClockServerProperties(const std::string& port_);

			bool IsInitialized();

			DTime GetOffset(Time t = Now());

			std::string GetPort() const;

			void UpdateOffset(Offset::OffsetMeasResult meas);
		};

	protected:
		virtual Offset::OffsetMeasResult DetermineOffset(const std::string& address, long long n) = 0;

	private:
		std::map<std::string, std::shared_ptr<ClockServerProperties>> clockOffsets; // address (192.168.0.1) without port -> offset properties
		bool toStop;
		std::thread t;
		std::mutex mutexForClockOffsets;

		void mainThread();

	public:
		ClockSyncronizerClient();

		void SynchronizePeriphery(const std::string& address, const std::string& port); /*!< Add Servers to be synchronized to */

		void SynchronizePeriphery(const NetworkConfig::ConnectionData& config);

		bool IsClockSynchronisationInProgress(const std::string& address); /*!< Check if synchronisation for a given device is in progress */

		DTime GetOffset(const std::string& address, Time time = Now()); /*!< Get the computed offset (0) if it is not computed*/

		void PrintStatus(); /*!< Print the actual status of the synchronizer client */

		~ClockSyncronizerClient();
	};

	ClockSyncronizerClient* GetPeripheryClockSynchronizerPtr(); /*!< Get the pointer of the statically inicialized implemented instance */

}
