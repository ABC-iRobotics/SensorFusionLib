#pragma once
#include <map>
#include <string>

namespace SF {

	class NetworkConfig {
	public:
		struct ConnectionData {
			std::string commType; // tcp/ipc
			std::string address; // IP/ipc_name
			std::string port;
			int hwm; // -1: not defined, 0: infinite, >0: the given value...
			ConnectionData(const std::string& type, const std::string& address_, const std::string& port_ = "");
			void SetHWM(int hwm_);
			std::string RecieverAddress() const;
			std::string SenderAddress() const;
		};

		std::map<std::string, ConnectionData> clockSyncData;

		std::map<std::string, ConnectionData> peripheryData;

		NetworkConfig(const std::string& filename);

		void Print() const;

		ConnectionData GetPeripheryData(const std::string& name) const;

		ConnectionData GetClockSyncData(const std::string& name) const;
	};

}