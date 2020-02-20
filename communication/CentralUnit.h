#pragma once
#include "Application.h"

namespace SF {

	/*! \brief Class to recieve DataMsgs from zmq publishers and writes them into an spdlog file
	*
	* The class initializes a zmq context and a subscriber socket, and save the recieved data via sdp logging
	*/
	/*!< Constructor: initializes a zmq context and a publisher socket ("tcp://*:15555" or ipc:///tmp/feeds/0 ...) and spd log*/
	class CentralUnit : public Application {
	public:
		CentralUnit(Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>());

		CentralUnit(const std::string& logfilename, Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>());

		CentralUnit(const std::string& socketaddressforresults, int hwm, Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>());

		void AddPeriphery(const Reciever::PeripheryProperties& prop); /*!< Add peripheries for networked recievers */

		void Start(DTime Ts);

		void Stop();

	protected:
		~CentralUnit();
	};


	/*! \brief Class to recieve DataMsgs from zmq publishers and writes them into an spdlog file
	*
	* The class initializes a zmq context and a subscriber socket, and save the recieved data via sdp logging
	*/
	class CentralUnitEmulator : public Application {
	public:
		CentralUnitEmulator(Processor::ProcessorPtr processor,
			const std::string& inputlogfilename);


		CentralUnitEmulator(const std::string& outputlogfilename, Processor::ProcessorPtr processor,
			const std::string& inputlogfilename);

		CentralUnitEmulator(const std::string& socketaddressforresults, int hwm, Processor::ProcessorPtr processor,
			const std::string& inputlogfilename);

		void Start(DTime Ts);

		void Stop();

	protected:
		~CentralUnitEmulator();
	};
}
