#pragma once
#include <zmq.hpp>
#include <thread>

namespace SF {

	/*! \brief 
	* 
	* 
	*/
	class ClockSynchronizerServer {
		std::thread serverthread;
		zmq::context_t context;
		zmq::socket_t socket;
		bool running;
	public: 
		ClockSynchronizerServer(const std::string& address); // eg: "tcp://*:5555"

		void StartServer();

		void StopServer();

		~ClockSynchronizerServer();

	private:
		void Run();
	};

	// vagy DTime?
	long long GetOffsetFromServerTime(std::string address, long long n_msgs = 10000, zmq::context_t context = zmq::context_t(1));
}
