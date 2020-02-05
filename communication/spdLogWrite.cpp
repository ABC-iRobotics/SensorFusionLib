#include "spdLogWrite.h"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include"spdlog/details/os.h"
#include"spdlog/details/fmt_helper.h"
#include"spdlog/spdlog.h"

using namespace SF;

spdLogWrite::spdLogWrite(std::string filename, std::string loggername) :
	my_logger(spdlog::basic_logger_mt<spdlog::async_factory>(loggername, filename)) {
	my_logger->set_pattern("[%Y/%m/%d-%H:%M:%S-%f] [%^%L%$] %v");
}

spdLogWrite::~spdLogWrite() {
	spdlog::drop(my_logger->name());
}

void spdLogWrite::WriteDataMsg(const DataMsg & msg) {
	buf.clear();
	fmt::format_to(buf, "MSG [ ");
	switch (msg.GetDataSourceType()) {
	case OperationType::FILTER_MEAS_UPDATE:
		fmt::format_to(buf, "DISTURBANCE");
		break;
	case OperationType::FILTER_PARAM_ESTIMATION:
		fmt::format_to(buf, "FILTER_PARAM_ESTIMATION");
		break;
	case OperationType::FILTER_TIME_UPDATE:
		fmt::format_to(buf, "FILTER_TIME_UPDATE");
		break;
	case OperationType::GROUND_TRUTH:
		fmt::format_to(buf, "GROUND_TRUTH");
		break;
	case OperationType::SENSOR:
		fmt::format_to(buf, "SENSOR");
		break;
	default:
		fmt::format_to(buf, "INVALID_OPERATIONTYPE");
		break;
	}
	fmt::format_to(buf, " {} ",msg.GetSourceID());
	switch (msg.GetDataType()) {
	case DataType::DISTURBANCE:
		fmt::format_to(buf, "DISTURBANCE");
		break;
	case DataType::STATE:
		fmt::format_to(buf, "STATE");
		break;
	case DataType::OUTPUT:
		fmt::format_to(buf, "OUTPUT");
		break;
	case DataType::NOISE:
		fmt::format_to(buf, "NOISE");
		break;
	default:
		fmt::format_to(buf, "INVALID_DATATYPE");
		break;
	}
	fmt::format_to(buf, " ");
	auto micros = spdlog::details::fmt_helper::time_fraction<std::chrono::microseconds>(msg.GetTime()).count();
	auto tm = spdlog::details::os::localtime(std::chrono::system_clock::to_time_t(msg.GetTime()));
	fmt::format_to(buf, "{}/", static_cast<unsigned int>(1900 + tm.tm_year));
	spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(1 + tm.tm_mon), buf);
	fmt::format_to(buf, "/");
	spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_mday), buf);
	fmt::format_to(buf, "-");
	spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_hour), buf);
	fmt::format_to(buf, ":");
	spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_min), buf);
	fmt::format_to(buf, ":");
	spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_sec), buf);
	fmt::format_to(buf, "-");
	spdlog::details::fmt_helper::pad6(static_cast<unsigned int>(micros), buf);
	fmt::format_to(buf, " ");
	if (msg.HasValue()) {
		Eigen::VectorXd v = msg.GetValue();
		fmt::format_to(buf, "VAL {} ", v.size());
		for (int i = 0; i < v.size(); i++)
			fmt::format_to(buf, "{} ", v[i]);
	}
	else
		fmt::format_to(buf, "NOVAL ");
	if (msg.HasVariance()) {
		Eigen::MatrixXd m = msg.GetVariance();
		fmt::format_to(buf, "VAR {} ", m.rows());
		for (int i = 0; i < m.cols(); i++)
			for (int j = i; j < m.rows(); j++)
				fmt::format_to(buf, "{} ", m(i, j));
	}
	else
		fmt::format_to(buf, "NOVAR ");
	fmt::format_to(buf, "]");
	my_logger->info(spdlog::string_view_t(buf.data(), buf.size()));
	buf.clear();
}
