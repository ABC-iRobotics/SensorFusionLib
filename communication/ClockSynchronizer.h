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

	/*! \brief Class to store the result of offset measurement
	*
	*
	*/
	class Offset {
	public:
		/*! \brief Struct of numeric results
		*/
		struct OffsetMeasResult {
			Time t; //!< Time of measurement
			DTime offset;  //!< Measured offset
			OffsetMeasResult(Time t_, DTime offset_); //!< Constructor
			OffsetMeasResult() {}; //!< Empty constructor
		};

		Offset() {} //!< Empty constructor

		Offset(OffsetMeasResult res1, OffsetMeasResult res2); //!< Constructor

		DTime GetOffset(Time t);  //!< Get computed offset

		void Add(OffsetMeasResult value); //!< Add measurement

		bool IsInitialized() const; //!< To check if class is initialized
	
	private:
		std::vector<OffsetMeasResult> values;
		Time t0;
		DTime offset0;
		double m;
		bool messy;

		void Update();
	};

	/*! \brief std::exception for communicating clock sync problems
	*
	*
	*/
	class ClockSyncConnectionError : public std::exception {
	private:
		std::string msg;
	public:
		ClockSyncConnectionError(std::string address); //!< Constructor

		virtual const char* what() const throw() override; //!< Desription of the error
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
			ClockServerProperties(const std::string& port_); //<! Constructor

			bool IsInitialized();

			DTime GetOffset(Time t = Now());

			std::string GetPort() const;

			void UpdateOffset(Offset::OffsetMeasResult meas);
		};

	protected:
		virtual Offset::OffsetMeasResult DetermineOffset(const std::string& address, long long n) = 0; //!< virtual method of a measurement

	private:
		std::map<std::string, std::shared_ptr<ClockServerProperties>> clockOffsets; // address (192.168.0.1) without port -> offset properties
		bool toStop;
		std::thread t;
		std::mutex mutexForClockOffsets;

		void mainThread();

	public:
		ClockSyncronizerClient();

		void SynchronizePeriphery(const std::string& address, const std::string& port); /*!< Add a server to be synchronized to */

		void SynchronizePeriphery(const NetworkConfig::ConnectionData& config); //!< Add a server to be synchronized to 

		bool IsClockSynchronisationInProgress(const std::string& address); /*!< Check if synchronisation for a given device is in progress */

		DTime GetOffset(const std::string& address, Time time = Now()); /*!< Get the computed offset (0) if it is not computed*/

		void PrintStatus(); /*!< Print the actual status of the synchronizer client */

		~ClockSyncronizerClient(); //!< Destructor
	};

	ClockSyncronizerClient* GetPeripheryClockSynchronizerPtr(); /*!< Get the pointer of the statically inicialized implemented instance */

}
