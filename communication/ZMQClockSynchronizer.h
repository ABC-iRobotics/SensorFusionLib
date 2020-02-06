#pragma once
#include <thread>
#include <chrono>

namespace SF {

	/*! \brief 
	* 
	* 
	*/
	class ZMQClockSynchronizerServer {
		std::string address;
		std::thread serverthread;
		bool running;
	public: 
		ZMQClockSynchronizerServer(const std::string& address_=""); // eg: "tcp://*:5555"

		void SetAddress(const std::string& address_);

		void StartServer();

		void StopServer();

		bool IsRunning() const;

		~ZMQClockSynchronizerServer();

	private:
		void Run();
	};

	std::chrono::nanoseconds DetermineClockOffsetFromZMQServer(const std::string& address, long long n_msgs = 10000, int zmq_io_threads = 1);

	class ClockSynchronizerClient {
		std::chrono::nanoseconds offset;
	public:
		ClockSynchronizerClient() : offset(0) {}

		void UpdateOffsetFromServerTime(const std::string& address, long long n_msgs = 10000, int zmq_io_threads = 1);

		std::chrono::nanoseconds GetOffset() const;
	};
	
}
