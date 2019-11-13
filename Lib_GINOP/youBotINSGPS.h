#pragma once
#include "FilteringManager.h"

/*! \brief An implementation of FilteringManager that initilizes a youBot model with INS and GPS sensors
*
*
*/
class youBotINSGPS : private FilteringManager {
public:
	youBotINSGPS(std::string loggeraddress = ""); /*!< Constructor */

	~youBotINSGPS(); /*!< Destructor */

	using FilteringManager::run;

	using FilteringManager::addSensorSockets;
};

