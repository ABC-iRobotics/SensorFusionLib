#pragma once
#include "DataMsg.h"

namespace SF {

	/*! \brief Abstract class for log reading in general
	*
	* Implementations: specified for SPDLog...
	*/
	class ILogReader {
	public:
		enum RowTypes /*!< Possible types od the rows */
			{DATAMSG, TEXT, NOTLOADED};

		virtual bool readNextRow() = 0; /*!< To read the next row from the log */

		virtual RowTypes getLatestRowType() const = 0; /*!< Get the type of the latest row read */

		virtual Time getLatestTimeStamp() const = 0; /*!< Get the timestamp of the latest row read */

		virtual const std::string& getLatestRowIf() const = 0; /*!< Get the std::string read last time */

		virtual const DataMsg& getLatestDataMsgIf() const = 0; /*!< Get the DataMsg read last time */

		typedef std::shared_ptr<ILogReader> ILogReaderPtr; /*!< std::shared_ptr for ILogReader */
	};
}