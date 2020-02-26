#include "Reciever.h"

using namespace SF;

const DTime Reciever::tWaitNextMsg = DTime(150);

SF::Reciever::Reciever()
	: isRunning(false) {}

void SF::Reciever::Start(DTime Ts)
{
	if (!isRunning) {
		isRunning = true;
		toStop = false;
		t = std::thread([this, Ts]() {
			_Run(Ts);
			isRunning = false;
		});
	}
}

void SF::Reciever::Stop(bool waitin)
{
	toStop = true;
	if (waitin) {
		while (isRunning)
			std::this_thread::sleep_for(std::chrono::milliseconds(5)); // or use mutexes...
		if (t.joinable())
			t.join();
	}
}

bool SF::Reciever::MustStop() const {
	return toStop;
}

bool SF::Reciever::IsRunning() const { return isRunning; }

void SF::Sender::CallbackSamplingTimeOver(const Time & currentTime) {}

void SF::Sender::CallbackGotDataMsg(const DataMsg & msg, const Time & currentTime) {
	msg.print();
}

void SF::Sender::CallbackGotString(const std::string & msg, const Time & currentTime) {
	std::cout << msg << std::endl;
}

void SF::Sender::CallbackMsgQueueEmpty(const Time & currentTime) {}
