#pragma once
#include "spdlog/logger.h"
#include<fstream>
#include "comm_defs.h"
#include "DataMsg.h"

namespace SF {

	/*! \brief Class for reading spd log
	*
	*/
	class SPDLogReader {
	public:
		SPDLogReader(std::string filename); /*!< Constructor */

		MsgType readNextRow(); /*!< Read the next row, returns its type */

		MsgType getLatestRowType() const;  /*!< Get the type of the latest row read */

		Time getLatestTimeStamp() const; /*!< Get the timestamp of the latest row read */

		const std::string& getLatestRowIf() const; /*!< Get the std::string read last time */

		const DataMsg& getLatestDataMsgIf() const; /*!< Get the DataMsg read last time */

		typedef std::shared_ptr<SPDLogReader> LogReaderPtr; /*!< std::shared_ptr for ILogReader */

	private:
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		MsgType latestRowType;
		std::string latestRow;
		static const int BUFSIZE = 200;
		char buf[BUFSIZE];
	};
}