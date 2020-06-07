#include "SteppableSimulator.h"

using namespace SF;

void SF::SteppableSimulator::_run(DTime Ts) {
	Time tLast, tNext;
	bool got, first = true;
	while (true) {
		// Read the next row
		if (!first)
			logread.readNextRow();
		// Initialize the variables
		if (first) {
			tLast = logread.getLatestTimeStamp();
			tNext = tLast + Ts;
			first = false;
			got = false;
		}
		// Inner iteration
		while (true) {
			// exit condition
			if (MustStop() || logread.getLatestRowType() == NOTHING)
				return;
			// Sampling ended
			if (logread.getLatestTimeStamp() > tNext && (tNext < tLast + tWaitNextMsg || !got)) {
				filterCore->SamplingTimeOver(tNext);
				// TODO: Send state and output via 
				tNext += Ts;
				got = false;
				continue;
			}
			// Read queue is empty
			if (got && (logread.getLatestTimeStamp() > tLast + tWaitNextMsg)) {
				filterCore->MsgQueueEmpty(tLast + tWaitNextMsg);
				got = false;
				continue;
			}
			break;
		}
		// Process the current row
		switch (logread.getLatestRowType()) {
		case DATAMSG:
			filterCore->SaveDataMsg(logread.getLatestDataMsgIf(), logread.getLatestTimeStamp()); // Send to the filter
			ForwardDataMsg(logread.getLatestDataMsgIf(), logread.getLatestTimeStamp()); // Send to the logger / zmq output
			got = true;
			break;
		case TEXT:
			ForwardString(logread.getLatestRowIf(), logread.getLatestTimeStamp());
			break;
		}
		tLast = logread.getLatestTimeStamp();
	}
}

SF::SteppableSimulator::SteppableSimulator(const char * filename, FilterCore::FilterCorePtr filterCore_)
	: Forwarder(), logread(filename), isRunning(false), filterCore(filterCore_) {}

void SF::SteppableSimulator::Start(DTime Ts) {
	if (!isRunning) {
		isRunning = true;
		toStop = false;
		t = std::thread([this, Ts]() {
			_run(Ts);
			isRunning = false;
		});
	}
}

void SF::SteppableSimulator::Stop(bool waitin) {
	toStop = true;
	if (waitin) {
		while (isRunning)
			std::this_thread::sleep_for(std::chrono::milliseconds(5)); // or use mutexes...
		if (t.joinable())
			t.join();
	}
}

bool SF::SteppableSimulator::MustStop() const {
	return toStop;
}

bool SF::SteppableSimulator::IsRunning() const { return isRunning; }

SF::SteppableSimulator::~SteppableSimulator() {
	Stop();
}
