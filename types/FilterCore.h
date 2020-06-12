#pragma once

#include"DataMsg.h"
#include"defs.h"

namespace SF {

	/*! \brief Prototype of core of the filters
	*
	*
	*
	*/
	class FilterCore {
	public:
		typedef std::shared_ptr<FilterCore> FilterCorePtr; //!< Pointer to the abstract class type

		virtual void SaveDataMsg(const DataMsg& msg, const Time& currentTime) = 0; /*!< Must called if new DataMsg recieved */

		virtual void SamplingTimeOver(const Time& currentTime) = 0; /*!< Is called in each sampling time - input: time */

		virtual void MsgQueueEmpty(const Time& currentTime) = 0; /*!< Is called if the DataMsgs in the queue were read */

		virtual DataMsg GetDataByID(int systemID, DataType dataType, OperationType opType) = 0; //!< To get predicted/filtered state/output/.. values

		virtual DataMsg GetDataByIndex(int systemIndex, DataType dataType, OperationType opType) = 0; //!< To get predicted/filtered state/output/.. values
	};
}
