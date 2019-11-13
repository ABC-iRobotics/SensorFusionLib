#include "youBotINSGPS.h"

int main(void) {
	youBotINSGPS filter("tcp://*:5554");
	filter.addSensorSockets("tcp://localhost:5555");
	filter.addSensorSockets("tcp://localhost:5556");
	filter.addSensorSockets("tcp://localhost:5557");
	filter.run();

	return 0;
}