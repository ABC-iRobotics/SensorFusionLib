#include "NetworkConfig.h"
#include <fstream>
#include <iostream>
#include "json.hpp"

using namespace SF;

NetworkConfig::ConnectionData::ConnectionData(const std::string & type, const std::string & address_, const std::string & port_) :
	commType(type), address(address_), port(port_), hwm(-1) {}

void SF::NetworkConfig::ConnectionData::SetHWM(int hwm_) {
	hwm = hwm_;
}

std::string NetworkConfig::ConnectionData::RecieverAddress() const {
	if (commType.compare("tcp") == 0)
		return "tcp://" + address + ":" + port;
	if (commType.compare("ipc") == 0)
		return "ipc:///" + address;
}

std::string NetworkConfig::ConnectionData::SenderAddress() const {
	if (commType.compare("tcp") == 0)
		return "tcp://*:" + port;
	if (commType.compare("ipc") == 0)
		return "ipc:///" + address;
}

void NetworkConfig::Add(const std::string& filename) {
	std::ifstream configFile;
	configFile.open(filename, std::ios_base::in);

	if (!configFile.is_open())
		throw std::runtime_error("FATAL ERROR: cannot open json file '" + filename + "' (in NetworkConfig::Add)");

	nlohmann::json config;
	configFile >> config;

	if (config.find("Remote") != config.end())
		for (auto remote : config.at("Remote").items()) {
			auto remote_ = remote.value();
			std::string IP = remote_.at("IP");
			if (remote_.find("ClockServerPort") != remote_.end())
				clockSyncData.insert(std::pair<std::string, ConnectionData>(remote.key(), ConnectionData("tcp", IP, remote_.at("ClockServerPort"))));
			auto peripheries = remote_.at("Peripheries");
			for (auto periphery : peripheries.items()) {
				peripheryData.insert(std::pair<std::string, ConnectionData>(periphery.key(), ConnectionData("tcp", IP, periphery.value().at("Port"))));
				if (periphery.value().find("hwm") != periphery.value().end())
					peripheryData.at(periphery.key()).SetHWM(periphery.value().at("hwm"));
			}

		}

	if (config.find("LocalPeripheries") != config.end())
		for (auto periphery : config.at("LocalPeripheries").items()) {
			std::string type = periphery.value().at("type");
			std::string address, port = "";
			if (type.compare("tcp") == 0) {
				address = "localhost";
				port = periphery.value().at("address");
			}
			if (type.compare("ipc") == 0)
				address = periphery.value().at("address");
			peripheryData.insert(std::pair<std::string, ConnectionData>(periphery.key(),
				ConnectionData(type, address, port)));
			if (periphery.value().find("hwm") != periphery.value().end())
				peripheryData.at(periphery.key()).SetHWM(periphery.value().at("hwm"));
		}
}

void SF::NetworkConfig::Add(const std::string & name, const ConnectionData & connection) {
	peripheryData.insert(std::pair<std::string, ConnectionData>(name, connection));
}

void NetworkConfig::Print() const {
	std::cout << "Network config:\n\n Clockservers:\n";
	for (auto it : clockSyncData)
		std::cout << it.first << " <- Recv: " << it.second.RecieverAddress() << " Send: " << it.second.SenderAddress() << std::endl;
	std::cout << "\n Peripheries:\n";
	for (auto it : peripheryData)
		std::cout << it.first << " <- Recv: " << it.second.RecieverAddress() << " Send: " << it.second.SenderAddress() << "  HWM: " << it.second.hwm << std::endl;
}

NetworkConfig::ConnectionData NetworkConfig::GetPeripheryData(const std::string & name) const {
	return peripheryData.at(name);
}

NetworkConfig::ConnectionData NetworkConfig::GetClockSyncData(const std::string & name) const {
	return clockSyncData.at(name);
}
