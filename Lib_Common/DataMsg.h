#pragma once

#include "defs.h"
#include "TimeMicroSec.h"
#include "StatisticValue.h"

class DataMsg {
	DataType dataType;
	OperationType dataSource;
	bool empty;
	unsigned char sourceID;
	Eigen::VectorXd value;
	bool hasValue;
	Eigen::MatrixXd variance;
	bool hasVariance;
	TimeMicroSec time;

public:
	DataMsg();

	DataMsg(unsigned char ID, DataType type, OperationType source, TimeMicroSec time_ = TimeMicroSec());

	DataMsg(unsigned char ID, DataType type, OperationType source,
		StatisticValue data, TimeMicroSec time_ = TimeMicroSec());;

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

	TimeMicroSec GetTime() const;

	void print() const;

	void SetVarianceMatrix(const Eigen::MatrixXd& m);

	void SetValueVector(const Eigen::VectorXd& v);
};

