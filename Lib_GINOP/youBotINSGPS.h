#pragma once
#include "FilteringManager.h"

/*! \brief An implementation of FilteringManager that initilizes a youBot model with INS and GPS sensors
*
*
*/
class youBotINSGPS : private FilteringManager {
public:
	youBotINSGPS(int port, int port_logger = -1); /*!< Constructor */

	~youBotINSGPS(); /*!< Destructor */

	using FilteringManager::run;
};

