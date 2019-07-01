
#include "ZMQSubscriber.h"
#include "FilterPlot.h"
#include "youBotSystem.h"
#include "INSSensor.h"

int main(void) {
	int youbotID = 0;
	int insID = 2;
	int gpsID = 1;

	youBotSystem::BaseSystemPtr youBot = std::make_shared<youBotSystem>(0, 0, 0, 0, youbotID);
	INSSensor::SensorPtr ins = std::make_shared<INSSensor>(youBot, insID);

	FilterPlot plotter(youbotID, youBot->getName(), youBot->getStateNames(), STATE);
	//FilterPlot plotter2(youbotID, youBot->getName(), youBot->getDisturbanceNames(), DISTURBANCE);
	//FilterPlot plotter3(insID, ins->getName(), ins->getOutputNames(), OUTPUT);


	ZMQSubscriber sub(5556);

	while (true) {
		DataMsg data;
		bool success = sub.RecvMsg_Wait(data);
		while (success) {
			FilterPlot::AddData(data);
			success = sub.RecvMsg_DontWait(data);
		}
		FilterPlot::UpdateWindows();
	}
}