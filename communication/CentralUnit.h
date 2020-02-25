#pragma once
#include "Application.h"
#include "NetworkConfig.h"

namespace SF {

	/*! \brief Class to connect to peripheries, process the msgs and save the output into a logfile or publish it on a socket
	*
	*
	*/
	class CentralUnit : public Application {
	public:
		CentralUnit(Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>()); /*!< Constructor to create an application with given processor to recieve data from peripheries process them */

		CentralUnit(const std::string& logfilename, Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>()); /*!< Constructor to create an application with given processor to recieve data from peripheries process them and save the output into a logfile */

		CentralUnit(const std::string& socketaddressforresults, int hwm, Processor::ProcessorPtr processor,
			const std::vector<Reciever::PeripheryProperties>& peripheries
			= std::vector<Reciever::PeripheryProperties>()); /*!< Constructor to create an application with given processor to recieve data from peripheries and publish its output via a ZMQ socket */

		void AddPeriphery(const Reciever::PeripheryProperties& prop); /*!< Add peripheries for networked recievers */

		void AddPeripheries(const NetworkConfig& config);

		void Start(DTime Ts); /*!< Start recieving and processing thread with given sampling time */

		void Stop(); /*!< Stop recieving and processing thread */

		~CentralUnit(); /*!< Destructor */
	};


	/*! \brief Class to emulate the behavior of a central unit from a log: processors can be tested, the output can be written into a logfile or published into a socket
	*
	* 
	*/
	class CentralUnitEmulator : public Application {
	public:
		CentralUnitEmulator(bool realtime, Processor::ProcessorPtr processor,
			const std::string& inputlogfilename); /*!< Constructor to create an application with given processor to read data from logfile and process them */

		CentralUnitEmulator(bool realtime, const std::string& outputlogfilename, Processor::ProcessorPtr processor,
			const std::string& inputlogfilename); /*!< Constructor to create an application with given processor to read data from logfile and process them, the output is logged*/

		CentralUnitEmulator(bool realtime, const std::string& socketaddressforresults, int hwm, Processor::ProcessorPtr processor,
			const std::string& inputlogfilename); /*!< Constructor to create an application with given processor to read data from logfile and process them, the output is published on a socket */

		void Start(DTime Ts); /*!< Start recieving and processing thread with given sampling time */

		void Stop();  /*!< Stop recieving and processing thread */

		~CentralUnitEmulator(); /*!< Destructor */
	};
}
