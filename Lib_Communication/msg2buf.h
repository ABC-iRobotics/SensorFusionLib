#pragma once

#include "DataMsg.h"

unsigned long getTimeInMicroseconds();

class Buffer {
public:
	Buffer();
	Buffer(const unsigned char * buf_, size_t size_);
	Buffer(const Buffer& buf0);
	Buffer(Buffer&& o);

	Buffer(const DataMsg& data);

	DataMsg ExtractDataMsg() const;
	
	~Buffer();

	Buffer& operator=(const Buffer& buf0);

	bool isNull() const;

	void print() const;

	size_t Size() const;

	const unsigned char* Buf() const;

private:
	void _allocandcopy(const unsigned char * buf_);

	unsigned char* buf;

	size_t size;
};
/*
class SystemDataMsg {
public:
	enum ContentTypes {
		TOFILTER_MEASUREMENT = 0,
		TOFILTER_DISTURBANCE = 1,
		FROMFILTER_PREDICTEDSTATE = 2,
		FROMFILTER_FILTEREDSTATE = 3,
		FROMFILTER_PREDICTEDOUTPUT = 4,
		FROMFILTER_MEASUREDOUTPUT = 5,
		FROMFILTER_USEDDISTURBANCE = 6,
		FROMFILTER_USEDNOISE = 7,
		EMPTY = 8
	};

	void print() const;

	SystemDataMsg(unsigned char ID, ContentTypes type, unsigned long timestamp_in_us);

	SystemDataMsg(); // to initialize empty instances

	void SetVarianceMatrix(const Eigen::MatrixXd& m);

	void SetValueVector(const Eigen::VectorXd& v);

	Buffer GetMsgBuffer() const;

	SystemDataMsg(const Buffer& buf);

	bool IsEmpty() const;

	bool IsToFilter() const;

	bool IsFromFilter() const;

	bool HasValue() const;

	bool HasVariance() const;

	Eigen::VectorXd Value() const;

	Eigen::MatrixXd Variance() const;

	unsigned char SourceID() const;

	ContentTypes ContentType() const;

private:
	unsigned char sourceID;
	Eigen::VectorXd value;
	bool hasValue;
	Eigen::MatrixXd variance;
	bool hasVariance;
	unsigned long timestamp_in_us;
	ContentTypes contentType;
};*/