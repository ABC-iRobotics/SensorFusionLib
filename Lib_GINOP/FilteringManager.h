#pragma once
#include "ZMQSubscriber.h"
#include "ZMQPublisher.h"
#include "SystemManager.h"

/*! \brief Class that recieves the zmq msgs forward them into a filter that is stepped in Ts_max or when new msg arrives
*
* Initialization:
* - Constuctor with appropriate settings
* - Set the filter to be used (SetFilter())
*  - Optionally set a zmq logger that sends msgs about the actual state (time/measurement updates): SetZMQLogger()
*
* Start it with the run() function
*/
class FilteringManager {
	ZMQRTSubscriber zmqSub;
	ZMQPublisher* zmqPub;
	const double Ts_max; // in seconds
	SystemManager::SystemManagerPtr filter;

	void _processMSG(const DataMsg & data);

	bool _getandprocessMsg_Wait(unsigned int waitinms = 50);

	bool _getandprocessMsg_DontWait();

	void _readLoadedZMQMsgs();

public:
	FilteringManager(double Ts_max_s); /*!< Constructor */

	void addSensorSockets(std::string address); // e.g. "tcp://10.8.0.14:5555" or "ipc:///tmp/feeds/0"

	void SetFilter(SystemManager::SystemManagerPtr filter_); /*!< Set filter - necessary before run()*/

	void SetZMQLogger(std::string loggeraddress); /*!< Set port to log the filtering results real-time */

	void run(); /*!< Start the filtering function*/

	~FilteringManager(); /*!< Destructor */
};
