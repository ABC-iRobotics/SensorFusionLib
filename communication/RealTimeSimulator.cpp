#include "RealTimeSimulator.h"

using namespace SF;

void SF::RealTimeSimulator::_run(DTime Ts) {
	bool got = false, first = true;
	DTime offset = duration_cast(Now() - logread.getLatestTimeStamp());
	std::function<Time()> Now2 = [offset]() { return Now() - offset; };
	Time tNext = logread.getLatestTimeStamp() + Ts;
	while (true) {
		// Read the next row
		if (!first)
			logread.readNextRow();
		else
			first = false;
		// Inner iteration
		while (true) {
			// exit condition
			if (MustStop() || logread.getLatestRowType() == NOTHING)
				return;
			Time deadline = Now2() + tWaitNextMsg;
			// Sampling ended
			if ((deadline > tNext) || (logread.getLatestTimeStamp() > tNext && !got)) {
				while (Now2() < tNext)
					;
				auto t = Now2();
				filterCore->SamplingTimeOver(t);
				// forward filtered state
				for (int i = 0; i < filterCore->nSensors() + 1; i++)
					ForwardDataMsg(filterCore->GetDataByIndex(i - 1, DataType::STATE, OperationType::FILTER_MEAS_UPDATE), t);
				// set variables
				tNext += Ts;
				got = false;
				continue;
			}
			// Read queue is empty
			if (got && (logread.getLatestTimeStamp() > deadline)) {
				while (Now2() < deadline)
					;
				auto t = Now2();
				filterCore->MsgQueueEmpty(t);
				// forward filtered state
				for (int i = 0; i < filterCore->nSensors() + 1; i++)
					ForwardDataMsg(filterCore->GetDataByIndex(i - 1, DataType::STATE, OperationType::FILTER_MEAS_UPDATE), t);
				// set variables
				tNext = t + Ts;
				got = false;
				continue;
			}
			break;
		}
		while (Now2() < logread.getLatestTimeStamp())
			;
		// Process the current row
		switch (logread.getLatestRowType()) {
		case DATAMSG:
			got |= filterCore->SaveDataMsg(logread.getLatestDataMsgIf(), Now2());
			ForwardDataMsg(logread.getLatestDataMsgIf(), Now2());
			break;
		case TEXT:
			ForwardString(logread.getLatestRowIf(), Now2());
			break;
		}
	}
}

SF::RealTimeSimulator::RealTimeSimulator(const std::string& filename, FilterCore::FilterCorePtr filterCorePtr)
	: Forwarder(), logread(filename), isRunning(false), filterCore(filterCorePtr) {}

void SF::RealTimeSimulator::Start(DTime Ts) {
	if (!isRunning) {
		isRunning = true;
		toStop = false;
		t = std::thread([this, Ts]() {
			_run(Ts);
			isRunning = false;
		});
	}
}

void SF::RealTimeSimulator::Stop(bool waitin) {
	toStop = true;
	if (waitin) {
		while (isRunning)
			std::this_thread::sleep_for(std::chrono::milliseconds(5)); // or use mutexes...
		if (t.joinable())
			t.join();
	}
}

bool SF::RealTimeSimulator::MustStop() const {
	return toStop;
}

bool SF::RealTimeSimulator::IsRunning() const { return isRunning; }

SF::RealTimeSimulator::~RealTimeSimulator() {
	Stop();
}