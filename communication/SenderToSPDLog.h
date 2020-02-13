#pragma once
#include "spdlog/logger.h"
#include "Application.h"

namespace SF {

	/*! \brief Sender implementation for reading from spd log
	*
	*/
	class SenderToSPDLog : public Sender {
		std::shared_ptr<spdlog::logger> my_logger;

		spdlog::memory_buf_t buf;

	public:
		SenderToSPDLog(std::string filename, std::string loggername); /*!< Constructor */

		~SenderToSPDLog();  /*!< Destructor */

		SenderToSPDLog(const SenderToSPDLog&) = delete;

		void SendDataMsg(const DataMsg& data) override;

		void SendString(const std::string& str) override;
	};

}