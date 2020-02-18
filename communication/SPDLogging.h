#pragma once
#include "spdlog/logger.h"
#include<fstream>
#include "Application.h"

namespace SF {

	/*! \brief Class for reading spd log
	*
	*/
	class SPDLogRead {
	public:
		enum RowTypes /*!< Possible types od the rows */
		{
			DATAMSG, TEXT, NOTLOADED
		};

		SPDLogRead(std::string filename); /*!< Constructor */

		bool readNextRow();

		RowTypes getLatestRowType() const;

		Time getLatestTimeStamp() const;

		const std::string& getLatestRowIf() const;

		const DataMsg& getLatestDataMsgIf() const;

	private:
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		RowTypes latestRowType;
		std::string latestRow;
		static const int BUFSIZE = 200;
		char buf[BUFSIZE];
	};

	/*! \brief Sender implementation for reading from spd log
	*
	*/
	class SPDSender : public Sender {
		std::shared_ptr<spdlog::logger> my_logger;

		spdlog::memory_buf_t buf;

	public:
		SPDSender(std::string filename, std::string loggername); /*!< Constructor */

		~SPDSender();  /*!< Destructor */

		SPDSender(const SPDSender&) = delete;

		void SendDataMsg(const DataMsg& data) override;

		void SendString(const std::string& str) override;
	};

}