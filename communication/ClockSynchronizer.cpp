#include "ClockSynchronizer.h"
#include<iostream>
#include<thread>
using namespace SF;

SF::ClockSyncronizerClient::PublisherClockProperties::PublisherClockProperties(std::string port_) :
	offset(Now(),DTime(0),0), inprogress(true), port(port_) {}

void SF::ClockSyncronizerClient::PublisherClockProperties::Set(PublisherClockProperties::Offset offset_) {
	inprogress = false;
	offset = offset_;
}

ClockSynchronizerServer::ClockSynchronizerServerPtr SF::InitClockSynchronizerServer(const NetworkConfig::ConnectionData& config) {
	return InitClockSynchronizerServer(config.SenderAddress());
}

void SF::ClockSyncronizerClient::Run() {
	bool found = true;
	while (found) {
		// look for new problem to syncronize
		mutexForClockOffsets.lock();
		std::string address;
		std::string port;
		found = false;
		//std::_Tree_iterator<std::map<std::string, std::shared_ptr<PublisherClockProperties>> element_;
		//std::pair<std::string, std::shared_ptr<PublisherClockProperties>> element_;
		for (auto element : clockOffsets)
			if (element.second->inprogress) {
				address = element.first;
				port = element.second->port;
				found = true;
				break;
			}
		mutexForClockOffsets.unlock();
		// if found
		if (found) {
			// synchronize
			auto offset_ = SynchronizeClock("tcp://" + address + ":" + port);
			// set result
			mutexForClockOffsets.lock();
			clockOffsets.at(address)->Set(offset_);
			mutexForClockOffsets.unlock();
		}
	}
	isRunning = false;
}

SF::ClockSyncronizerClient::ClockSyncronizerClient() : clockOffsets(std::map<std::string, std::shared_ptr<PublisherClockProperties>>()),
isRunning(false) {}

void SF::ClockSyncronizerClient::SynchronizePeriphery(const std::string & address, const std::string& port) {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets)
		if (element.first.find(address) != std::string::npos) {
			mutexForClockOffsets.unlock();
			return;
		}
	clockOffsets.insert(std::pair<std::string, std::shared_ptr<PublisherClockProperties>>(address,
		std::make_shared<PublisherClockProperties>(port)));
	mutexForClockOffsets.unlock();
	
	if (!isRunning) {
		isRunning = true;
		std::thread t(&ClockSyncronizerClient::Run, this);
		t.detach();
	}
}

void SF::ClockSyncronizerClient::SynchronizePeriphery(const NetworkConfig::ConnectionData & config) {
	SynchronizePeriphery(config.address, config.port);
}

bool SF::ClockSyncronizerClient::IsClockSynchronisationInProgress(const std::string& address) { //address: tcp://192.168.0.1:1234
	mutexForClockOffsets.lock();
	bool out = false;
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = element.second->inprogress;
	mutexForClockOffsets.unlock();
	return out;
}

DTime SF::ClockSyncronizerClient::GetOffset(const std::string& address, Time time) {
	mutexForClockOffsets.lock();
	DTime out(0);
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = element.second->offset.Value(time);
	mutexForClockOffsets.unlock();
	return out;
}

void SF::ClockSyncronizerClient::PrintStatus() {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets) {
		std::cout << element.first << "(" << element.second->port << ")";
		if (element.second->inprogress)
			std::cout << "? ";
		else
			std::cout << element.second->offset.offset0.count() << " us (at " <<
			duration_cast(element.second->offset.time0.time_since_epoch()).count() << " us) m = " << element.second->offset.m;
	}
	std::cout << std::endl;
	mutexForClockOffsets.unlock();
}
