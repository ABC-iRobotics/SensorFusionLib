
#include "ZMQSubscriber.h"
#include "FilterPlot.h"

int main(void) {
	int youbotID = 0;
	int insID = 2;
	int gpsID = 1;

	FilterPlot plotter(youbotID, "youBot", {"vxlocal","vylocal","omega","x","y","phi"}, STATE);
	//FilterPlot plotter2(youbotID, youBot->getName(), youBot->getDisturbanceNames(), DISTURBANCE);
	//FilterPlot plotter3(insID, ins->getName(), ins->getOutputNames(), OUTPUT);


	ZMQLogSubscriber sub;
	sub.addSocket("tcp://localhost:5554");
	DataMsg data;

	while (true) {
		auto start = std::chrono::system_clock::now();
		while (std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now() - start).count() < 50) {
			if (sub.RecvMsg_Wait(data, 100))
				FilterPlot::AddDataToWindows(data);
		}
		FilterPlot::UpdateWindows();
	}
}