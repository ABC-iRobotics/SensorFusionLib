#include "ClockSynchronizer.h"
#include<iostream>
#include<exception>
using namespace SF;


ClockSynchronizerServer::ClockSynchronizerServerPtr SF::InitClockSynchronizerServer(const NetworkConfig::ConnectionData& config) {
	return InitClockSynchronizerServer(config.SenderAddress());
}

SF::Offset::OffsetMeasResult::OffsetMeasResult(Time t_, DTime offset_) : offset(offset_), t(t_) {}

void SF::Offset::Update() {
	if (values.size() == 0)
		throw std::runtime_error("Update on empty offset calculator object");
	long long t0_ = 0;
	long long offset0_ = 0;
	for (auto it : values) {
		t0_ += duration_cast(it.t.time_since_epoch()).count();
		offset0_ += it.offset.count();
	}
	t0_ /= values.size();
	offset0_ /= (long long)values.size();
	long long cov = 0;
	long long var = 0;
	for (auto it : values) {
		cov += (duration_cast(it.t.time_since_epoch()).count() - t0_) * (it.offset.count() - offset0_);
		var += (duration_cast(it.t.time_since_epoch()).count() - t0_) * (duration_cast(it.t.time_since_epoch()).count() - t0_);
	}
	m = double(cov) / double(var);
	//std::cout << m << std::endl;
	t0 = Time(DTime(t0_));
	offset0 = DTime(offset0_);
	messy = false;
}

SF::Offset::Offset(OffsetMeasResult res1, OffsetMeasResult res2) : values(std::vector<OffsetMeasResult>()) {
	Add(res1);
	Add(res2);
}

DTime SF::Offset::GetOffset(Time t) {
	if (messy)
		Update();
	return offset0 + duration_cast((t - t0) * m);
}

void SF::Offset::Add(OffsetMeasResult value) {
	bool toCheck = IsInitialized();
	Time t = Now();
	DTime offset0;
	if (toCheck)
		offset0 = GetOffset(t);

	if (values.size() < 30)
		values.push_back(value);
	else
		values[value.t.time_since_epoch().count() % 30] = value;
	messy = true;

	if (toCheck) {
		DTime diff = offset0 - GetOffset(t);
		if ( abs(diff.count()) * values.size() > 300*30 )
			std::cout << "Warning the offset update caused large difference: " << diff.count() << " us " << std::endl;
	}
}

bool SF::Offset::IsInitialized() const {
	return values.size() >= 2;
}

SF::ClockSyncConnectionError::ClockSyncConnectionError(std::string address) : msg("Timeout connecting to the server '" + address + "'!") {}

const char* ClockSyncConnectionError::what() const throw() {
	return msg.c_str();
}

SF::ClockSyncronizerClient::ClockServerProperties::ClockServerProperties(const std::string & port_) : port(port_), offset() {}

bool SF::ClockSyncronizerClient::ClockServerProperties::IsInitialized() {
	bool out;
	guard.lock();
	out = offset.IsInitialized();
	guard.unlock();
	return out;
}

DTime SF::ClockSyncronizerClient::ClockServerProperties::GetOffset(Time t) {
	DTime out(0);
	guard.lock();
	if (offset.IsInitialized())
		out = offset.GetOffset(t);
	guard.unlock();
	return out;
}

std::string SF::ClockSyncronizerClient::ClockServerProperties::GetPort() const {
	return port;
}

void SF::ClockSyncronizerClient::ClockServerProperties::UpdateOffset(Offset::OffsetMeasResult meas) {
	guard.lock();
	offset.Add(meas);
	guard.unlock();
}

void SF::ClockSyncronizerClient::mainThread() {

	DTime Ts = duration_cast(std::chrono::seconds(3));
	Time tnext = Now() + Ts;
	while (!toStop) {
		bool toUpdateAll = false;
		if (Now() > tnext) {
			tnext += Ts;
			toUpdateAll = true;
		}

		// Check for a new device
		mutexForClockOffsets.lock();
		for (auto& it : clockOffsets)
			if (!it.second->IsInitialized() || toUpdateAll) {
				std::string address = "tcp://" + it.first + ":" + it.second->GetPort();
				try {
					do {
						mutexForClockOffsets.unlock();
						auto toSet = DetermineOffset(address, 200);
						mutexForClockOffsets.lock();
						it.second->UpdateOffset(toSet);
					} while (!it.second->IsInitialized());
				}
				catch (ClockSyncConnectionError& e) {
					std::cout << "Warning: unable to connect to ClockServer\n" << e.what() << std::endl;
					mutexForClockOffsets.lock();
					continue;
				}
			}
		mutexForClockOffsets.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
	}
}

SF::ClockSyncronizerClient::ClockSyncronizerClient() : clockOffsets(std::map<std::string, std::shared_ptr<ClockServerProperties>>()),
toStop(false) {
	t = std::thread([this]() { mainThread(); });
}

void SF::ClockSyncronizerClient::SynchronizePeriphery(const std::string & address, const std::string & port) {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets)
		if (element.first.find(address) != std::string::npos) {
			mutexForClockOffsets.unlock();
			return;
		}
	clockOffsets.insert(std::pair<std::string, std::shared_ptr<ClockServerProperties>>(address,
		std::make_shared<ClockServerProperties>(port)));
	mutexForClockOffsets.unlock();
}

void SF::ClockSyncronizerClient::SynchronizePeriphery(const NetworkConfig::ConnectionData & config) {
	SynchronizePeriphery(config.address, config.port);
}

bool SF::ClockSyncronizerClient::IsClockSynchronisationInProgress(const std::string & address) {
	mutexForClockOffsets.lock();
	bool out = false;
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = !element.second->IsInitialized();
	mutexForClockOffsets.unlock();
	return out;
}

DTime SF::ClockSyncronizerClient::GetOffset(const std::string & address, Time time) {
	mutexForClockOffsets.lock();
	DTime out(0);
	for (auto element : clockOffsets)
		if (address.find(element.first) != std::string::npos)
			out = element.second->GetOffset(time);
	mutexForClockOffsets.unlock();
	return out;
}

void SF::ClockSyncronizerClient::PrintStatus() {
	mutexForClockOffsets.lock();
	for (auto element : clockOffsets) {
		std::cout << element.first << "(" << element.second->GetPort() << ")";
		if (!element.second->IsInitialized())
			std::cout << "? ";
		else
			std::cout << element.second->GetOffset().count() << " us \n";
	}
	std::cout << std::endl;
	mutexForClockOffsets.unlock();
}

SF::ClockSyncronizerClient::~ClockSyncronizerClient() {
	toStop = true;
	t.join();
}
