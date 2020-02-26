#pragma once
#include "NetworkConfig.h"
#include "Reciever.h"

namespace SF {

	/*! \brief Class to recieve DataMsgs from zmq publishers and writes them into an spdlog file
	*
	* The class initializes a zmq context and a subscriber socket, and save the recieved data via sdp logging
	*/
	class Logger {
		Sender::SenderPtr sender;
		Reciever::RecieverPtr reciever;

	public:
		Logger(const std::string& filename); /*!< Constructor: initializes a zmq context and a publisher socket ("tcp://*:15555" or ipc:///tmp/feeds/0 ...) and spd log*/

		void AddPeripheries(const NetworkConfig& config);

		void Start(DTime Ts); /*!< Start recieving thread with given sampling time */

		void Stop(); /*!< Stop recieving thread */

		~Logger(); /*!< Destructor */
	};
}
