#pragma once
#include "spdlog/logger.h"
#include "DataMsg.h"
#include<string>

namespace SF {

	class spdLogWrite {

		std::shared_ptr<spdlog::logger> my_logger;

		spdlog::memory_buf_t buf;

	public:
		spdLogWrite(std::string filename, std::string loggername);

		~spdLogWrite();

		spdLogWrite(const spdLogWrite&) = delete;

		void WriteDataMsg(const DataMsg& data);
	};

}