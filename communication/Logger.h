#pragma once
#include "Application.h"

namespace SF {

	/*! \brief Class to recieve DataMsgs from zmq publishers and writes them into an spdlog file
	*
	* The class initializes a zmq context and a subscriber socket, and save the recieved data via sdp logging
	*/
	class Logger : public Application {
	public:
		Logger(const std::string& filename,
			const std::vector<Reciever::PeripheryProperties>& peripheries = std::vector<Reciever::PeripheryProperties>()); /*!< Constructor: initializes a zmq context and a publisher socket ("tcp://*:15555" or ipc:///tmp/feeds/0 ...) and spd log*/

		void AddPeriphery(const Reciever::PeripheryProperties& prop); /*!< Add peripheries for networked recievers */
	};
}
