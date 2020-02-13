#pragma once
#include"ILogReader.h"
#include<fstream>

namespace SF {

	/*! \brief Class for reading spd log
	*
	*/
	class SPDLogRead : public ILogReader {
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		RowTypes latestRowType;
		std::string latestRow;
		static const int BUFSIZE = 200;
		char buf[BUFSIZE];

	public:
		SPDLogRead(std::string filename); /*!< Constructor */

		bool readNextRow() override;

		RowTypes getLatestRowType() const override;

		Time getLatestTimeStamp() const override;

		const std::string& getLatestRowIf() const override;

		const DataMsg& getLatestDataMsgIf() const override;
	};
}
