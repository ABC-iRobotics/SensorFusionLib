#include "msg2buf.h"
#include "flatbuffers/flatbuffers.h"
#include "msg_generated.h"
#include <iostream>


#include <windows.h>

unsigned long getTimeInMicroseconds() {
	SYSTEMTIME time;
	GetSystemTime(&time);
	unsigned long out = time.wYear;
	out *= 12;
	out += time.wDay;
	out *= 24;
	out += time.wHour;
	out *= 60;
	out += time.wMinute;
	out *= 60;
	out += time.wSecond;
	out *= 1000;
	out += time.wMilliseconds;
	out *= 1000;
	return out;
}


Buffer::Buffer(const unsigned char * buf_, size_t size_) :
	size(size_) {
	_allocandcopy(buf_);
}

Buffer::Buffer(const Buffer & buf_) :
	size(buf_.size) {
	_allocandcopy(buf_.Buf());
}

Buffer::Buffer(Buffer && o) : size(std::move(o.size)) { _allocandcopy(o.Buf()); }

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

void DataMsg::print() const {
	unsigned int t = getTimeInMicroseconds();
	if (contentType == EMPTY) {
		printf("EMTPY DataMsg.\n\n");
		return;
	}
	printf("SourceID: %d Content: ", sourceID);
	switch (contentType)
	{
	case MEASUREMENT:
		printf("MEASUREMENT");
		break;
	case DISTURBANCE:
		printf("DISTURBANCE");
		break;
	}
	printf(" Age: %f [ms]\n", (float)(t - timestamp_in_us)*1.e-3);
	if (isValue)
		std::cout << "Value:\n" << value << std::endl;
	if (isVariance)
		std::cout << "Variance:\n" << variance << std::endl;
	printf("\n");
}

void DataMsg::SetVarianceMatrix(Eigen::MatrixXd m) {
	variance = m;
	isVariance = true;
}

void DataMsg::SetValueVector(Eigen::VectorXd v) {
	value = v;
	isValue = true;
}

DataMsg::DataMsg(unsigned char ID, ContentType type, unsigned long timestamp_in_us_) :
	sourceID(ID), contentType(type), isValue(false), isVariance(false), timestamp_in_us(timestamp_in_us_) {
	if (type==DataMsg::EMPTY)
		throw std::runtime_error(std::string("DataMsg::DataMsg wrong argument"));
}

DataMsg::DataMsg() : contentType(EMPTY), isValue(false), isVariance(false) {}

DataMsg::DataMsg(const Buffer & buf) {
	auto msg = SensorDataMsg::GetMsg(buf.Buf());

	isValue = flatbuffers::IsFieldPresent(msg, SensorDataMsg::Msg::VT_VALUE_VECTOR);
	if (isValue) {
		auto v = msg->value_vector();
		size_t N = v->size();
		value = Eigen::VectorXd(N);
		for (unsigned int n = 0; n < N; n++)
			value[n] = v->Get(n);
	}

	isVariance = flatbuffers::IsFieldPresent(msg, SensorDataMsg::Msg::VT_VARIANCE_MATRIX);
	if (isVariance) {
		auto v = msg->variance_matrix();
		size_t K = v->size();
		size_t N = static_cast<size_t>((sqrt(8 * K + 1) - 1) / 2);
		variance = Eigen::MatrixXd(N, N);
		int k = 0;
		for (unsigned int i = 0; i < N; i++)
			for (unsigned int j = i; j < N; j++) {
				variance(i, j) = v->Get(k);
				variance(j, i) = variance(i, j);
				k++;
			}
	}

	switch (msg->type()) {
	case SensorDataMsg::ValueType::ValueType_MEASUREMENT:
		contentType = MEASUREMENT;
		break;
	case SensorDataMsg::ValueType::ValueType_DISTURBANCE:
		contentType = DISTURBANCE;
		break;
	}

	timestamp_in_us = msg->timestamp_in_us();

	sourceID = msg->sensorID();
}

Buffer DataMsg::GetMsgBuffer() const {
	flatbuffers::FlatBufferBuilder fbb(1024);

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_value;
	if (isValue) {
		size_t N = value.size();
		float* v = new float[N];
		for (unsigned int i = 0; i < N; i++)
			v[i] = (float)value[i];
		fbb_value = fbb.CreateVector<float>(v, 4);
	}

	flatbuffers::Offset<flatbuffers::Vector<float>> fbb_variance;
	if (isVariance) {
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

	SensorDataMsg::MsgBuilder msgBuilder(fbb);
	switch (contentType)
	{
	case MEASUREMENT:
		msgBuilder.add_type(SensorDataMsg::ValueType_MEASUREMENT);
		break;
	case DISTURBANCE:
		msgBuilder.add_type(SensorDataMsg::ValueType_DISTURBANCE);
		break;
	}

	msgBuilder.add_sensorID(sourceID);

	if (isValue)
		msgBuilder.add_value_vector(fbb_value);

	if (isVariance)
		msgBuilder.add_variance_matrix(fbb_variance);

	msgBuilder.add_timestamp_in_us(timestamp_in_us);

	auto msg = msgBuilder.Finish();
	fbb.Finish(msg);

	return Buffer(fbb.GetBufferPointer(), fbb.GetSize());
}
