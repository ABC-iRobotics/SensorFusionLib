#pragma once
#include "spdlog/logger.h"
#include<fstream>
#include "Application.h"

namespace SF {

	/*! \brief Class for reading spd log
	*
	*/
	class SPDLogReader {
	public:
		enum RowType /*!< Possible types od the rows */
		{
			DATAMSG, TEXT, NOTLOADED
		};

		SPDLogReader(std::string filename); /*!< Constructor */

		RowType readNextRow(); /*!< Read the next row, returns its type */

		RowType getLatestRowType() const;  /*!< Get the type of the latest row read */

		Time getLatestTimeStamp() const; /*!< Get the timestamp of the latest row read */

		const std::string& getLatestRowIf() const; /*!< Get the std::string read last time */

		const DataMsg& getLatestDataMsgIf() const; /*!< Get the DataMsg read last time */

		typedef std::shared_ptr<SPDLogReader> LogReaderPtr; /*!< std::shared_ptr for ILogReader */
	private:
		std::ifstream stream;
		Time firstTime;
		Time latestTime;
		DataMsg latestMsg;
		RowType latestRowType;
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