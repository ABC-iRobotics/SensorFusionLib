#include "msg2buf.h"
#include <flatbuffers/flatbuffers.h>
#include "msg_generated.h"
#include <iostream>

#include "TimeMicroSec.h"

Buffer::Buffer(const unsigned char * buf_, size_t size_) :
	size(size_) {
	_allocandcopy(buf_);
}

Buffer::Buffer(const Buffer & buf_) :
	size(buf_.size) {
	_allocandcopy(buf_.Buf());
}

Buffer::Buffer(Buffer && o) : size(std::move(o.size)) { _allocandcopy(o.Buf()); }

Buffer::Buffer(const DataMsg & data) {
	flatbuffers::FlatBufferBuilder fbb(1024);

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_value;
	if (data.HasValue()) {
		auto value = data.GetValue();
		size_t N = value.size();
		float* v = new float[N];
		for (unsigned int i = 0; i < N; i++)
			v[i] = (float)value[i];
		fbb_value = fbb.CreateVector<float>(v, N);
	}

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_variance;
	if (data.HasVariance()) {
		auto variance = data.GetVariance();
		unsigned int N = static_cast<unsigned int>(variance.rows());
		unsigned int K = static_cast<unsigned int>((N*(N + 1)) / 2);
		float* v = new float[K];
		int k = 0;
		for (unsigned int i = 0; i < N; i++)
			for (unsigned int j = i; j < N; j++) {
				v[k] = (float)variance(i, j);
				k++;
			}
		fbb_variance = fbb.CreateVector<float>(v, K);
	}

	DataMsgNameSpace::MsgBuilder msgBuilder(fbb);
	switch (data.GetDataType()) {
	case STATE:
		msgBuilder.add_type(DataMsgNameSpace::DataType_STATE);
		break;
	case OUTPUT:
		msgBuilder.add_type(DataMsgNameSpace::DataType_OUTPUT);
		break;
	case NOISE:
		msgBuilder.add_type(DataMsgNameSpace::DataType_NOISE);
		break;
	case DISTURBANCE:
		msgBuilder.add_type(DataMsgNameSpace::DataType_DISTURBANCE);
		break;
	}
	switch (data.GetDataSourceType()) {
	case FILTER_TIME_UPDATE:
		msgBuilder.add_source(DataMsgNameSpace::OperationType_FILTER_TIME_UPDATE);
		break;
	case FILTER_MEAS_UPDATE:
		msgBuilder.add_source(DataMsgNameSpace::OperationType_FILTER_MEAS_UPDATE);
		break;
	case FILTER_PARAM_ESTIMATION:
		msgBuilder.add_source(DataMsgNameSpace::OperationType_FILTER_PARAM_ESTIMATION);
		break;
	case SENSOR:
		msgBuilder.add_source(DataMsgNameSpace::OperationType_SENSOR);
		break;
	}

	msgBuilder.add_sensorID(data.GetSourceID());

	if (data.HasValue())
		msgBuilder.add_value_vector(fbb_value);

	if (data.HasVariance())
		msgBuilder.add_variance_matrix(fbb_variance);

	msgBuilder.add_timestamp_in_us(data.GetTime().TimeInUS());

	auto msg = msgBuilder.Finish();
	fbb.Finish(msg);

	size = fbb.GetSize();

	_allocandcopy(fbb.GetBufferPointer());
}

Buffer::Buffer() : size(0), buf(NULL) {}

bool Buffer::isNull() const { return size == 0; }

void Buffer::print() const {
	std::cout << "Size: " << size << std::endl;
	for (size_t i = 0; i < size; i++)
		printf(" %i\n", buf[i]);
}

size_t Buffer::Size() const { return size; }

const unsigned char* Buffer::Buf() const { return buf; }

void Buffer::_allocandcopy(const unsigned char * buf_) {
	buf = new unsigned char[size];
	for (size_t i = 0; i < size; i++)
		buf[i] = buf_[i];
}

DataMsg Buffer::ExtractDataMsg() const {
	auto msg = DataMsgNameSpace::GetMsg(buf);

	DataType type;
	switch (msg->type()) {
	case DataMsgNameSpace::DataType_STATE:
		type = STATE;
		break;
	case DataMsgNameSpace::DataType_OUTPUT:
		type = OUTPUT;
		break;
	case DataMsgNameSpace::DataType_DISTURBANCE:
		type = DISTURBANCE;
		break;
	case DataMsgNameSpace::DataType_NOISE:
		type = NOISE;
		break;
	}

	OperationType source;
	switch (msg->source()) {
	case DataMsgNameSpace::OperationType_FILTER_TIME_UPDATE:
		source = FILTER_TIME_UPDATE;
		break;
	case DataMsgNameSpace::OperationType_FILTER_MEAS_UPDATE:
		source = FILTER_MEAS_UPDATE;
		break;
	case DataMsgNameSpace::OperationType_FILTER_PARAM_ESTIMATION:
		source = FILTER_PARAM_ESTIMATION;
		break;
	case DataMsgNameSpace::OperationType_SENSOR:
		source = SENSOR;
		break;
	}

	auto timestamp_in_us = msg->timestamp_in_us();

	auto sourceID = msg->sensorID();

	DataMsg out(sourceID, type, source, TimeMicroSec(timestamp_in_us));

	if (flatbuffers::IsFieldPresent(msg, DataMsgNameSpace::Msg::VT_VALUE_VECTOR)) {
		auto v = msg->value_vector();
		size_t N = v->size();
		auto value = Eigen::VectorXd(N);
		for (unsigned int n = 0; n < N; n++)
			value[n] = v->Get(n);
		out.SetValueVector(value);
	}

	if (flatbuffers::IsFieldPresent(msg, DataMsgNameSpace::Msg::VT_VARIANCE_MATRIX)) {
		auto v = msg->variance_matrix();
		size_t K = v->size();
		size_t N = static_cast<size_t>((sqrt(8 * K + 1) - 1) / 2);
		auto variance = Eigen::MatrixXd(N, N);
		int k = 0;
		for (unsigned int i = 0; i < N; i++)
			for (unsigned int j = i; j < N; j++) {
				variance(i, j) = v->Get(k);
				variance(j, i) = variance(i, j);
				k++;
			}
		out.SetVarianceMatrix(variance);
	}
	return out;
}

Buffer::~Buffer() { delete buf; }

Buffer& Buffer::operator=(const Buffer& buf0) {
	if (this != &buf0) {
		if (!isNull())
			delete buf;
		size = buf0.size;
		_allocandcopy(buf0.Buf());
	}
	return *this;
}