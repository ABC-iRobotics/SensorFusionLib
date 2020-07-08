#include "msg_old2buf.h"
#include <flatbuffers/flatbuffers.h>
#include "msg_old_generated.h"
#include <iostream>

using namespace SF;

Buffer_old::Buffer_old(const unsigned char * buf_, size_t size_) :
	size(size_) {
	_allocandcopy(buf_);
}

Buffer_old::Buffer_old(const Buffer_old & buf_) :
	size(buf_.size) {
	_allocandcopy(buf_.Buf());
}

Buffer_old::Buffer_old(Buffer_old && o) : size(std::move(o.size)) { _allocandcopy(o.Buf()); }

Buffer_old::Buffer_old(const DataMsg & data) {
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

	msgBuilder.add_timestamp_in_us(duration_since_epoch(data.GetTime()).count());

	auto msg = msgBuilder.Finish();
	fbb.Finish(msg);

	size = fbb.GetSize();

	_allocandcopy(fbb.GetBufferPointer());
}

Buffer_old::Buffer_old() : size(0), buf(NULL) {}

bool Buffer_old::isNull() const { return size == 0; }

void Buffer_old::print() const {
	std::cout << "Size: " << size << " - ";
	for (size_t i = 0; i < size; i++)
		printf("%i ", buf[i]);
	printf("\n");
}

size_t Buffer_old::Size() const { return size; }

const unsigned char* Buffer_old::Buf() const { return buf; }

void Buffer_old::_allocandcopy(const unsigned char * buf_) {
	buf = new unsigned char[size];
	for (size_t i = 0; i < size; i++)
		buf[i] = buf_[i];
}

bool SF::ExtractBufIf(void* buf, size_t size, DataMsg& data) {
	auto msg = DataMsgNameSpace::GetMsg(buf);
	flatbuffers::Verifier v((uint8_t*)buf, size);
	if (!msg->Verify(v))
		return false;
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
	default:
		type = INVALID_DATATYPE;
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
	default:
		source = INVALID_OPERATIONTYPE;
	}

	auto timestamp_in_us = msg->timestamp_in_us();

	auto sourceID = msg->sensorID();

	data = DataMsg(sourceID, type, source, Time(DTime(timestamp_in_us)));

	if (flatbuffers::IsFieldPresent(msg, DataMsgNameSpace::Msg::VT_VALUE_VECTOR)) {
		auto v = msg->value_vector();
		size_t N = v->size();
		auto value = Eigen::VectorXd(N);
		for (unsigned int n = 0; n < N; n++)
			value[n] = v->Get(n);
		data.SetValueVector(value);
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
		data.SetVarianceMatrix(variance);
	}
	return true;
}

Buffer_old::~Buffer_old() { delete buf; }

Buffer_old& Buffer_old::operator=(const Buffer_old& buf0) {
	if (this != &buf0) {
		if (!isNull())
			delete buf;
		size = buf0.size;
		_allocandcopy(buf0.Buf());
	}
	return *this;
}