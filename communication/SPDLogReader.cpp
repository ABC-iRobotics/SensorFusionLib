#include"SPDLogReader.h"
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
	temp.tm_isdst = 0;
	temp.tm_year = char2long(buf, 4) - 1900;
	temp.tm_mon = char2long(buf + 5, 2) - 1;
	temp.tm_mday = char2long(buf + 8, 2);
	temp.tm_hour = char2long(buf + 11, 2);
	temp.tm_min = char2long(buf + 14, 2);
	temp.tm_sec = char2long(buf + 17, 2);
	int micro = char2long(buf + 20, 6);
	return std::chrono::system_clock::from_time_t(std::mktime(&temp)) + std::chrono::microseconds(char2long(buf + 20, 6));
}

SPDLogReader::SPDLogReader(const std::string& filename) : stream(filename), latestRowType(NOTHING) {
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
		latestRowType = DATAMSG;
		return MsgType::DATAMSG;
	}
	else {
		stream.getline(buf, BUFSIZE, '\n');
		latestRow = buf;
		if (latestRow.size() == 0) {
			latestRowType = NOTHING;
			return MsgType::NOTHING;
		}
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
