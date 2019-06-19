#include "msg2buf.h"
#include <thread>
#include "ZMQSubscriber.h"

int main(void) {
	ZMQSubscriber sub;

	while (true) {
		DataMsg d;
		if (sub.RecvMsg_Wait(d))
			d.print();
	}

	/*
	while (true) {
		DataMsg d;
		sub.RecvString_DontWait();
		std::this_thread::sleep_for(std::chrono::duration<float, std::milli>(100.f));
	}*/
}