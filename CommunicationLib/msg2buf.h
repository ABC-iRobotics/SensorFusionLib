#pragma once

#include "Eigen/Dense"

unsigned long getTimeInMicroseconds();

const int port = 5556;

class Buffer {
public:
	Buffer();
	Buffer(const unsigned char * buf_, size_t size_);
	Buffer(const Buffer& buf0);
	Buffer(Buffer&& o);
	
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

struct DataMsg {
public:
	unsigned char sourceID;
	enum ContentType { MEASUREMENT, DISTURBANCE, EMPTY } contentType;
	Eigen::VectorXd value;
	bool isValue;
	Eigen::MatrixXd variance;
	bool isVariance;
	unsigned long timestamp_in_us;

	void print() const;

	DataMsg(unsigned char ID, ContentType type, unsigned long timestamp_in_us);

	DataMsg(); // to initialize empty instances

	void SetVarianceMatrix(Eigen::MatrixXd m);

	void SetValueVector(Eigen::VectorXd v);

	Buffer GetMsgBuffer() const;

	DataMsg(const Buffer& buf);
};
