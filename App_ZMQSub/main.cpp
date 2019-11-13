#include "ZMQSubscriber.h"

int main(void) {
	ZMQRTSubscriber sub;
	sub.addSocket("tcp://localhost:5555");
	sub.addSocket("tcp://localhost:5556");
	sub.addSocket("tcp://localhost:5557");

	while (true) {
		DataMsg data;
		if (sub.RecvMsg_Wait()) {
			for (unsigned int i = 0; i < sub.numSockets(); i++) {
				sub.getData(i, data);
				if (!data.IsEmpty())
					data.print();
			}
		}
	}

	/*
	while (true) {
		DataMsg d;
		sub.RecvString_DontWait();
		std::this_thread::sleep_for(std::chrono::duration<float, std::milli>(100.f));
	}*/
}