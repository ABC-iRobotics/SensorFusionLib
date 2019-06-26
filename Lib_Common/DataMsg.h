#pragma once

#include "defs.h"
#include "TimeUS.h"

class DataMsg {
	DataType dataType;
	OperationType dataSource;
	bool empty;
	unsigned char sourceID;
	Eigen::VectorXd value;
	bool hasValue;
	Eigen::MatrixXd variance;
	bool hasVariance;
	unsigned long timestamp_in_us;

public:
	DataMsg();

	DataMsg(unsigned char ID, DataType type, OperationType source, unsigned long timestamp_in_us_ = TimeUS().TimeInUS());;

	bool IsEmpty() const;

	bool HasValue() const;

	bool HasVariance() const;

	void ClearValue();

	void ClearVariance();

	Eigen::VectorXd GetValue() const;

	Eigen::MatrixXd GetVariance() const;

	unsigned char GetSourceID() const;

	DataType GetDataType() const;

	OperationType GetDataSourceType() const;

	unsigned long GetTimeInUs() const;

	void print() const;

	void SetVarianceMatrix(const Eigen::MatrixXd& m);

	void SetValueVector(const Eigen::VectorXd& v);
};

