#pragma once
#include <map>
#include <string>

namespace SF {

	/*! \brief Class that describes the addresses of the peripheries and the necessary properties
	*
	*
	*/
	class NetworkConfig {
	public:
		/*! \brief struct that describes the connection to a periphery
		*
		*
		*/
		struct ConnectionData {
			std::string commType; //!< tcp or ipc
			std::string address; //!< IP or ipc_name
			std::string port; //!< port for TCP
			int hwm; //!< -1: not defined, 0: infinite, >0: the given value...
			ConnectionData(const std::string& type, const std::string& address_, const std::string& port_ = "");
			//!< Constructor
			void SetHWM(int hwm_);  //!< to modify hwm settings
			std::string RecieverAddress() const; //!< get address can be used for the reciever
			std::string SenderAddress() const; //!< get address can be used for the sender
		};

		std::map<std::string, ConnectionData> clockSyncData; //!< std::map of clocksync configs

		std::map<std::string, ConnectionData> peripheryData; //!< std::map of connection configs

		void Add(const std::string& filename); //!< Add a network config file

		void Add(const std::string& name, const ConnectionData& connection); //!< Add a periphery settings with a name

		void Print() const; //!< Print the config

		ConnectionData GetPeripheryData(const std::string& name) const; //!< Get connection config of a given periphery

		ConnectionData GetClockSyncData(const std::string& name) const; //!< Get clocksync config of a given periphery
	};

}