#include"SPDLogging.h"
#include"spdlog/async.h"
#include"spdlog/sinks/basic_file_sink.h"
#include"spdlog/details/os.h"
#include"spdlog/details/fmt_helper.h"
#include"spdlog/spdlog.h"

using namespace SF;

DataType DataType2char(const char* buf) {
	switch (buf[0]) {
	case 'N':
		return DataType::NOISE;
	case 'O':
		return DataType::OUTPUT;
	case 'S':
		return DataType::STATE;
	case 'D':
		return DataType::DISTURBANCE;
	}
	return DataType::INVALID_DATATYPE;
}

OperationType OperationType2char(const char* buf) {
	switch (buf[0]) {
	case 'S':
		return OperationType::SENSOR;
	case 'G':
		return OperationType::GROUND_TRUTH;
	case 'F':
		switch (buf[7]) {
		case 'T':
			return OperationType::FILTER_TIME_UPDATE;
		case 'M':
			return OperationType::FILTER_MEAS_UPDATE;
		case 'P':
			return OperationType::FILTER_PARAM_ESTIMATION;
		}
	}
	return OperationType::INVALID_OPERATIONTYPE;
}

#include<ctime>

Time BuildTime(int year, int month, int day, int hour, int min, int sec, int micro) {
	std::tm temp;
	temp.tm_year = year - 1900;
	temp.tm_mon = month - 1;
	temp.tm_mday = day;
	temp.tm_hour = hour;
	temp.tm_min = min;
	temp.tm_sec = sec;
	return std::chrono::system_clock::from_time_t(std::mktime(&temp)) + std::chrono::microseconds(micro);
}

long char2long(char* buf, size_t length) {
	long out = 0;
	for (int i = 0; i < length; i++) {
		out += int(*(buf + i)) - 48;
		if (i + 1 < length)
			out *= 10;
	}
	return out;
}

Time readTime(char* buf) {
	std::tm temp;
	temp.tm_year = char2long(buf, 4) - 1900;
	temp.tm_mon = char2long(buf + 5, 2) - 1;
	temp.tm_mday = char2long(buf + 8, 2);
	temp.tm_hour = char2long(buf + 11, 2);
	temp.tm_min = char2long(buf + 14, 2);
	temp.tm_sec = char2long(buf + 17, 2);
	int micro = char2long(buf + 20, 6);
	return std::chrono::system_clock::from_time_t(std::mktime(&temp)) + std::chrono::microseconds(char2long(buf + 20, 6));
}

SPDLogReader::SPDLogReader(std::string filename) : stream(filename), latestRowType(NOTHING) {
	if (!stream.is_open())
		throw std::runtime_error(std::string("SPDLogReader::SPDLogReader File not found!"));
	readNextRow();
}

MsgType SPDLogReader::readNextRow() {
	if (stream.eof()) {
		latestRowType = NOTHING;
		return MsgType::NOTHING;
	}

	stream.getline(buf, BUFSIZE, ' '); // logging time
	latestTime = readTime(buf + 1);

	stream.getline(buf, BUFSIZE, ' '); //[I]

	stream.getline(buf, BUFSIZE, ' '); //MSG
	if (strcmp(buf, "MSG") == 0) {
		stream.getline(buf, BUFSIZE, ' '); //'['

		stream.getline(buf, BUFSIZE, ' '); // source
		OperationType source = OperationType2char(buf);

		stream.getline(buf, BUFSIZE, ' '); // sourceID
		long id = char2long(buf, strlen(buf));

		stream.getline(buf, BUFSIZE, ' '); // valuetype
		DataType type = DataType2char(buf);

		stream.getline(buf, BUFSIZE, ' '); // to use?
		Time msgTime = readTime(buf);

		latestMsg = DataMsg(static_cast<unsigned int>(id), type, source, msgTime);

		stream.getline(buf, BUFSIZE, ' ');
		if (strcmp(buf, "VAL") == 0) {
			stream.getline(buf, BUFSIZE, ' '); // N
			size_t n = char2long(buf, strlen(buf));
			Eigen::VectorXd v(n);
			for (size_t j = 0; j < n; j++) {
				stream.getline(buf, BUFSIZE, ' '); // v(j)
				v(j) = atof(buf);
			}
			latestMsg.SetValueVector(v);
		} // else: NOVAL...

		stream.getline(buf, BUFSIZE, ' ');
		if (strcmp(buf, "VAR") == 0) {
			stream.getline(buf, BUFSIZE, ' '); // N
			size_t n = char2long(buf, strlen(buf));
			Eigen::MatrixXd m(n, n);
			for (size_t j = 0; j < n; j++)
				for (size_t k = j; k < n; k++) {
					stream.getline(buf, BUFSIZE, ' '); // m(j,k),
					m(j, k) = atof(buf);
					m(k, j) = m(j, k);
				}
			latestMsg.SetVarianceMatrix(m);
		} // else: NOVAR...
		stream.ignore(100000, '\n');
		return MsgType::DATAMSG;
	}
	else {
		stream.getline(buf, BUFSIZE, '\n');
		latestRow = buf;
		latestRowType = TEXT;
		return MsgType::TEXT;
	}
}

MsgType SPDLogReader::getLatestRowType() const {
	return latestRowType;
}

Time SPDLogReader::getLatestTimeStamp() const {
	return latestTime;
}

const std::string & SPDLogReader::getLatestRowIf() const {
	return latestRow;
}

const DataMsg& SPDLogReader::getLatestDataMsgIf() const {
	return latestMsg;
}

SPDSender::SPDSender(std::string filename, std::string loggername) :
	my_logger(spdlog::basic_logger_mt<spdlog::async_factory>(loggername, filename)) {
	my_logger->set_pattern("[%Y/%m/%d-%H:%M:%S-%f] [%^%L%$] %v");
}

SPDSender::~SPDSender() {
	spdlog::drop(my_logger->name());
}

void SPDSender::SendDataMsg(const DataMsg & msg) {
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
	fmt::format_to(buf, " {} ", msg.GetSourceID());
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

void SF::SPDSender::SendString(const std::string & string) {
	my_logger->info(string.c_str());
}

/*! The core of the thread that reads the log and forwards it to the processor with the logged time stamps without checking the real time clocks
*
* \htmlonly
* <embed src="Reading log - steppable.pdf" width="700px" height="830px" href="Reading log (steppable mode)"></embed>
* \endhtmlonly
*/

void SF::SPDReciever::_Run(DTime Ts) {
	if (realtime)
		_RunRT(Ts);
	else
		_RunSteppable(Ts);
}


class LogSteppablePoller {
	SPDLogReader logread;
	bool readInAdvance;
	Time t;
public:
	DataMsg data;

	std::string string;

	MsgType type;

	LogSteppablePoller(const std::string& filename) : logread(filename), readInAdvance(true) {
		logread.readNextRow();
		t = logread.getLatestTimeStamp();
	}

	Time Now() const { return t; }

	bool eof() const { return logread.getLatestRowType() == NOTHING; }

	MsgType Poll(DTime dt) {
		DTime dt2 = dt + DTime(15);
		if (!readInAdvance) {
			logread.readNextRow();
		}
		if (logread.getLatestTimeStamp() > t + dt2) {
			readInAdvance = true;
			t += dt2;
			type = NOTHING;
			return type;
		}
		else {
			readInAdvance = false;
			t = logread.getLatestTimeStamp();
			type = logread.getLatestRowType();
			data = logread.getLatestDataMsgIf();
			string = logread.getLatestRowIf();
			return type;
		}
	}
};

void SF::SPDReciever::_RunSteppable(DTime Ts, DTime recieveTime) {
	DTime tRead(15);
	Time tLast, tNext;
	bool got, first = true;
	while (true) {
		// Read the next row
		logread.readNextRow();
		// Initialize the variables
		if (first) {
			tLast = logread.getLatestTimeStamp();
			tNext = tLast + Ts;
			first = false;
			got = false;
		}
		// Inner iteration
		while (true) {
			// exit condition
			if (MustStop() || logread.getLatestRowType() == NOTHING)
				return;
			// Sampling ended
			if (logread.getLatestTimeStamp() > tNext && (tNext<tLast+tRead || !got)) {
				CallbackSamplingTimeOver(tNext);
				tNext += Ts;
				got = false;
				continue;
			}
			// Read queue is empty
			if (got && (logread.getLatestTimeStamp() > tLast + tRead)) {
				CallbackMsgQueueEmpty(tLast + tRead);
				got = false;
				continue;
			}
			break;
		}
		// Process the current row
		switch (logread.getLatestRowType()) {
		case DATAMSG:
			CallbackGotDataMsg(logread.getLatestDataMsgIf(), logread.getLatestTimeStamp());
			got = true;
			break;
		case TEXT:
			CallbackGotString(logread.getLatestRowIf(), logread.getLatestTimeStamp());
			break;
		}
		tLast = logread.getLatestTimeStamp();
	}
}

/*! The core of the thread that reads the log and forwards it to the processor according to the real time clocks
*
* \htmlonly
* <embed src="Reading log - real time.pdf" width="700px" height="950px" href="Reading log - real time.pdf"></embed>
* \endhtmlonly
*/

void SF::SPDReciever::_RunRT(DTime Ts) {
	DTime tRead(15);
	Time tNext;
	//DTime offset;
	std::function<Time()> Now2;
	bool got, first = true;
	while (true) {
		// Read the next row
		logread.readNextRow();
		// Initialize the variables
		if (first) {
			DTime offset = duration_cast(Now() - logread.getLatestTimeStamp());
			Now2 = [offset]() { return Now() - offset; };
			tNext = logread.getLatestTimeStamp() + Ts;
			first = false;
			got = false;
		}
		// Inner iteration
		while (true) {
			// exit condition
			if (MustStop() || logread.getLatestRowType() == NOTHING)
				return;
			// Sampling ended
			if ((Now2() > tNext) || (logread.getLatestTimeStamp() > tNext && !got)) {
				if (Now2() < tNext)
					std::this_thread::sleep_for(tNext - Now2());
				CallbackSamplingTimeOver(Now2());
				tNext += Ts;
				got = false;
				continue;
			}
			// Read queue is empty
			if (got && (logread.getLatestTimeStamp() > Now2() + tRead)) {
				CallbackMsgQueueEmpty(Now2());
				got = false;
				continue;
			}
			break;
		}
		if (Now2() < logread.getLatestTimeStamp())
			std::this_thread::sleep_for(logread.getLatestTimeStamp() - Now2());
		// Process the current row
		switch (logread.getLatestRowType()) {
		case DATAMSG:
			CallbackGotDataMsg(logread.getLatestDataMsgIf(), Now2());
			got = true;
			break;
		case TEXT:
			CallbackGotString(logread.getLatestRowIf(), Now2());
			break;
		}
	}
}
