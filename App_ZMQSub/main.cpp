#include "ZMQSubscriber.h"

int main(void) {
	ZMQSubscriber sub(5555);

	while (true) {
		SystemDataMsg d;
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