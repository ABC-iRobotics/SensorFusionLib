#include "IClockSynchronizer.h"
#include<iostream>
#include<thread>
using namespace SF;

SF::IClockSyncronizerClient::PublisherClockProperties::PublisherClockProperties(std::string port_) :
	offset(0), inprogress(true), port(port_) {}

void SF::IClockSyncronizerClient::PublisherClockProperties::Set(DTime offset_) {
	inprogress = false;
	offset = offset_;
}

void SF::IClockSyncronizerClient::Run() {
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
			DTime offset = SynchronizeClock("tcp://" + address + ":" + port);
			// set result
			mutexForClockOffsets.lock();
			clockOffsets.at(address)->Set(offset);
			mutexForClockOffsets.unlock();
		}
	}
	isRunning = false;
}

SF::IClockSyncronizerClient::IClockSyncronizerClient() : clockOffsets(std::map<std::string, std::shared_ptr<PublisherClockProperties>>()),
isRunning(false) {}

void SF::IClockSyncronizerClient::SynchronizePeriphery(const std::string & address, const std::string& port) {
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
		std::thread t(&IClockSyncronizerClient::Run, this);
		t.detach();
	}
}

bool SF::IClockSyncronizerClient::IsClockSynchronisationInProgress(const std::string& address) { //address: tcp://192.168.0.1:1234
	mutexForClockOffsets.lock();
	bool out = false;
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = element.second->inprogress;
	mutexForClockOffsets.unlock();
	return out;
}

DTime SF::IClockSyncronizerClient::GetOffset(const std::string& address) {
	mutexForClockOffsets.lock();
	DTime out(0);
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = element.second->offset;
	mutexForClockOffsets.unlock();
	return out;
}

void SF::IClockSyncronizerClient::PrintStatus() {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets) {
		std::cout << element.first << "(" << element.second->port << ")";
		if (element.second->inprogress)
			std::cout << "? ";
		else
			std::cout << element.second->offset.count() << "us ";
	}
	std::cout << std::endl;
	mutexForClockOffsets.unlock();
}
