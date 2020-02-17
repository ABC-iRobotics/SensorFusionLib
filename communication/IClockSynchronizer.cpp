#include "IClockSynchronizer.h"
#include<iostream>
#include<thread>
using namespace SF;

SF::IClockSyncronizerClient::PublisherClockProperties::PublisherClockProperties(const std::string & address_) :
	offset(0), inprogress(true), address(address_) {}

void SF::IClockSyncronizerClient::PublisherClockProperties::Set(DTime offset_) {
	inprogress = false;
	offset = offset_;
}

void SF::IClockSyncronizerClient::Run() {
	while (true) {
		// look for new problem to syncronize
		mutexForClockOffsets.lock();
		std::string address;
		bool found = false;
		for (auto element : clockOffsets)
			if (element.second->inprogress) {
				address = element.second->address;
				found = true;
				break;
			}
		mutexForClockOffsets.unlock();
		// if found
		if (found) {
			// synchronize
			DTime offset = SynchronizeClock(address);
			// set result
			mutexForClockOffsets.lock();
			for (auto element : clockOffsets)
				if (element.second->address.compare(address) == 0) {
					std::cout << "Set: " << (int)element.first << std::endl;
					element.second->Set(offset);
				}
					
			mutexForClockOffsets.unlock();
		}
		else {
			isRunning = false;
			return;
		}
	}
}

SF::IClockSyncronizerClient::IClockSyncronizerClient() : clockOffsets(std::map<unsigned char, std::shared_ptr<PublisherClockProperties>>()),
isRunning(false) {}

void SF::IClockSyncronizerClient::SynchronizePeriphery(unsigned char ID, const std::string & address) {
	if (clockOffsets.find(ID) != clockOffsets.end())
		return;
	mutexForClockOffsets.lock();
	// iterate over addresses called by ClockSyncThreads and set the offsets with mutexes
	bool found = false;
	for (auto element : clockOffsets)
		if (element.second->address.compare(address) == 0) {
			clockOffsets.insert(std::pair<unsigned char, std::shared_ptr<PublisherClockProperties>>(ID, element.second));
			found = true;
			break;
		}
	if (!found)
		clockOffsets.insert(std::pair<unsigned char, std::shared_ptr<PublisherClockProperties>>(ID,
			std::make_shared<PublisherClockProperties>(address)));
	mutexForClockOffsets.unlock();
	if (!isRunning) {
		isRunning = true;
		std::thread t(&IClockSyncronizerClient::Run, this);
		t.detach();
	}
}

bool SF::IClockSyncronizerClient::IsClockSynchronisationInProgress(unsigned char ID) {
	mutexForClockOffsets.lock();
	auto it = clockOffsets.find(ID);
	bool out = false;
	if (it != clockOffsets.end())
		out = it->second->inprogress;
	mutexForClockOffsets.unlock();
	return out;
}

DTime SF::IClockSyncronizerClient::GetOffset(unsigned char ID) {
	mutexForClockOffsets.lock();
	auto it = clockOffsets.find(ID);
	DTime out(0);
	if (it != clockOffsets.end())
		out = it->second->offset;
	mutexForClockOffsets.unlock();
	return out;
}

void SF::IClockSyncronizerClient::PrintStatus() {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets) {
		std::cout << (int)element.first << "(" << element.second->address << ")";
		if (element.second->inprogress)
			std::cout << "? ";
		else
			std::cout << element.second->offset.count() << "us ";
	}
	std::cout << std::endl;
	mutexForClockOffsets.unlock();
}
