#include "Forwarder.h"

using namespace SF;

#include"spdlog/async.h"
#include"spdlog/sinks/basic_file_sink.h"
#include"spdlog/details/os.h"
#include"spdlog/details/fmt_helper.h"
#include"spdlog/spdlog.h"

#include"msgcontent2buf.h"
#include"zmq_addon.hpp"

SF::Forwarder::Forwarder() : spd_logger(NULL), zmq_context(NULL), zmq_socket(NULL) {}

SF::Forwarder::~Forwarder() {
	if (spd_logger)
		spdlog::drop(spd_logger->name());
	if (zmq_socket) {
		zmq_socket->close();
		zmq_context->close();
	}
}

void SF::Forwarder::SetLogger(const char * filename) {
	if (!spd_logger) {
		static int i = 0;
		spd_logger = spdlog::basic_logger_mt<spdlog::async_factory>(std::to_string(i), filename);
		i++;
		spd_logger->set_pattern("[%Y/%m/%d-%H:%M:%S-%f] [%^%L%$] %v");
	}
}

void SF::Forwarder::SetZMQOutput(const std::string & address, int hwm) {
	if (!zmq_socket) {
		zmq_context = std::make_shared<zmq::context_t>(2);
		zmq_socket = std::make_shared<zmq::socket_t>(*zmq_context, ZMQ_PUB);
		zmq_socket->setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
		try {
			zmq_socket->bind(address);
		}
		catch (const zmq::error_t& t) {
			zmq_socket->close();
			zmq_context->close();
			zmq_socket.reset();
			zmq_context.reset();
			zmq_socket = NULL;
			zmq_context = NULL;
			std::throw_with_nested(std::runtime_error("FATAL ERROR: ZMQ unable to bind to '" + address + "' (in ZMQSender::ZMQSender) (" + t.what() + ")"));
		}
	}
}

void SF::Forwarder::ForwardDataMsg(const DataMsg & msg, const Time & currentTime) {
	if (spd_logger) {
		spd_buf.clear();
		fmt::format_to(spd_buf, "MSG [ ");
		switch (msg.GetDataSourceType()) {
		case OperationType::FILTER_MEAS_UPDATE:
			fmt::format_to(spd_buf, "FILTER_MEAS_UPDATE");
			break;
		case OperationType::FILTER_PARAM_ESTIMATION:
			fmt::format_to(spd_buf, "FILTER_PARAM_ESTIMATION");
			break;
		case OperationType::FILTER_TIME_UPDATE:
			fmt::format_to(spd_buf, "FILTER_TIME_UPDATE");
			break;
		case OperationType::GROUND_TRUTH:
			fmt::format_to(spd_buf, "GROUND_TRUTH");
			break;
		case OperationType::SENSOR:
			fmt::format_to(spd_buf, "SENSOR");
			break;
		default:
			fmt::format_to(spd_buf, "INVALID_OPERATIONTYPE");
			break;
		}
		fmt::format_to(spd_buf, " {} ", msg.GetSourceID());
		switch (msg.GetDataType()) {
		case DataType::DISTURBANCE:
			fmt::format_to(spd_buf, "DISTURBANCE");
			break;
		case DataType::STATE:
			fmt::format_to(spd_buf, "STATE");
			break;
		case DataType::OUTPUT:
			fmt::format_to(spd_buf, "OUTPUT");
			break;
		case DataType::NOISE:
			fmt::format_to(spd_buf, "NOISE");
			break;
		default:
			fmt::format_to(spd_buf, "INVALID_DATATYPE");
			break;
		}
		fmt::format_to(spd_buf, " ");
		auto micros = spdlog::details::fmt_helper::time_fraction<std::chrono::microseconds>(msg.GetTime()).count();
		auto tm = spdlog::details::os::localtime(std::chrono::system_clock::to_time_t(msg.GetTime()));
		fmt::format_to(spd_buf, "{}/", static_cast<unsigned int>(1900 + tm.tm_year));
		spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(1 + tm.tm_mon), spd_buf);
		fmt::format_to(spd_buf, "/");
		spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_mday), spd_buf);
		fmt::format_to(spd_buf, "-");
		spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_hour), spd_buf);
		fmt::format_to(spd_buf, ":");
		spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_min), spd_buf);
		fmt::format_to(spd_buf, ":");
		spdlog::details::fmt_helper::pad2(static_cast<unsigned int>(tm.tm_sec), spd_buf);
		fmt::format_to(spd_buf, "-");
		spdlog::details::fmt_helper::pad6(static_cast<unsigned int>(micros), spd_buf);
		fmt::format_to(spd_buf, " ");
		if (msg.HasValue()) {
			Eigen::VectorXd v = msg.GetValue();
			fmt::format_to(spd_buf, "VAL {} ", v.size());
			for (int i = 0; i < v.size(); i++)
				fmt::format_to(spd_buf, "{} ", v[i]);
		}
		else
			fmt::format_to(spd_buf, "NOVAL ");
		if (msg.HasVariance()) {
			Eigen::MatrixXd m = msg.GetVariance();
			fmt::format_to(spd_buf, "VAR {} ", m.rows());
			for (int i = 0; i < m.cols(); i++)
				for (int j = i; j < m.rows(); j++)
					fmt::format_to(spd_buf, "{} ", m(i, j));
		}
		else
			fmt::format_to(spd_buf, "NOVAR ");
		fmt::format_to(spd_buf, "]");
		spd_logger->info(spdlog::string_view_t(spd_buf.data(), spd_buf.size()));
		spd_buf.clear();
	}
	if (zmq_socket) {
		zmq::multipart_t zmq_msg;
		unsigned char topicbuf[4];
		topicbuf[0] = 'd';
		topicbuf[1] = to_underlying<OperationType>(msg.GetDataSourceType());
		topicbuf[2] = msg.GetSourceID();
		topicbuf[3] = to_underlying<DataType>(msg.GetDataType());
		zmq_msg.addmem(topicbuf, 4);
		unsigned char* buf;
		int bufsize;
		SerializeDataMsg(msg, buf, bufsize);
		zmq_msg.addmem((void*)buf, bufsize);
		try {
			zmq_msg.send(*zmq_socket, ZMQ_DONTWAIT);
		}
		catch (...) {
			std::throw_with_nested(std::runtime_error("FATAL ERROR: sending ZMQ msg (MQSender::CallbackGotDataMsg)"));
		}
		delete buf;
	}
}

void SF::Forwarder::ForwardString(const std::string & msg, const Time & currentTime) {
	if (spd_logger) {
		spd_logger->info(("STR " + msg).c_str());
	}
	if (zmq_socket) {
		zmq::multipart_t zmq_msg;
		char i = 'i';
		zmq_msg.addmem(&i, 1);
		zmq_msg.addmem(&msg.c_str()[0], msg.length());
		try {
			zmq_msg.send(*zmq_socket, ZMQ_DONTWAIT);
		}
		catch (...) {
			std::throw_with_nested(std::runtime_error("FATAL ERROR: sending ZMQ msg (MQSender::CallbackGotString)"));
		}
	}
}
