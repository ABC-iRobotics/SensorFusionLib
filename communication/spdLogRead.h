#pragma once
#include<string>
#include"DataMsg.h"
#include<fstream>

namespace SF {

	class spdLogRead {
	public:
		enum RowTypes { DATAMSG, TEXT, NOTLOADED };

	private:
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		RowTypes latestRowType;
		std::string latestRow;
		static const int BUFSIZE = 200;
		char buf[BUFSIZE];

	public:
		spdLogRead(std::string filename);

		bool readNextRow();

		RowTypes getLatestRowType() const;

		Time getLatestTimeStamp() const;

		const std::string& getLatestRowIf() const;

		const DataMsg& getLatestDataMsgIf() const;
	};

}