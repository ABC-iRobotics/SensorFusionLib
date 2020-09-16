#include "DataMsg.h"
#include <iostream>

using namespace SF;

DataMsg::DataMsg(unsigned char ID, DataType type, OperationType source, StatisticValue data, const Time& time_) :
	sourceID(ID), dataType(type), dataSource(source), time(time_),
	hasValue(true), hasVariance(true), value(data.vector), variance(data.variance) {}

bool DataMsg::IsInvalid() const { return dataType==INVALID_DATATYPE || dataSource==INVALID_OPERATIONTYPE; }

bool DataMsg::HasValue() const { return hasValue; }

bool DataMsg::HasVariance() const { return hasVariance; }

void DataMsg::ClearValue() { hasValue = false; }

void DataMsg::ClearVariance() { hasVariance = false; }

Eigen::VectorXd DataMsg::GetValue() const { return value; }

Eigen::MatrixXd DataMsg::GetVariance() const { return variance; }

unsigned char DataMsg::GetSourceID() const { return sourceID; }

DataType DataMsg::GetDataType() const { return dataType; }

OperationType DataMsg::GetDataSourceType() const { return dataSource; }

void DataMsg::print() const {
	auto t = Now();
	if (IsInvalid()) {
		printf("EMTPY DataMsg.\n\n");
		return;
	}
	printf("Content: ");
	switch (dataType) {
	case STATE:
		printf("STATE");
		break;
	case OUTPUT:
		printf("OUTPUT");
		break;
	case NOISE:
		printf("NOISE");
		break;
	case DISTURBANCE:
		printf("DISTURBANCE");
		break;
	}
	printf(" from ");
	switch (dataSource) {
	case FILTER_TIME_UPDATE:
		printf("filter(time update) (ID: %d)", sourceID);
		break;
	case FILTER_MEAS_UPDATE:
		printf("filter(meas. update) (ID: %d)", sourceID);
		break;
	case FILTER_PARAM_ESTIMATION:
		printf("filter(param. estimation) (ID: %d)", sourceID);
		break;
	case SENSOR:
		printf("sensor (ID: %d) ", sourceID);
		break;
	default:
		break;
	}
	printf("\n TimeStamp: %lld  Age: %lld [us]\n", duration_cast(time.time_since_epoch()).count(), duration_cast(t - time).count());
	if (hasValue)
		std::cout << "Value:\n" << value << std::endl;
	if (hasVariance)
		std::cout << "Variance:\n" << variance << std::endl;
	printf("\n");
}

Time DataMsg::GetTime() const {
	return time;
}

DataMsg::DataMsg() // to initialize empty instances
	: hasValue(false), hasVariance(false), dataType(INVALID_DATATYPE), dataSource(INVALID_OPERATIONTYPE) {}

SF::DataMsg::DataMsg(unsigned char ID, DataType type, OperationType source, const Time & time_)
	: hasValue(false), hasVariance(false), dataType(type), dataSource(source), sourceID(ID), time(time_) {}

void DataMsg::SetVarianceMatrix(const Eigen::MatrixXd & m) {
	hasVariance = true;
	variance = m;
}

void DataMsg::SetValueVector(const Eigen::VectorXd & v) {
	hasValue = true;
	value = v;
}

void DataMsg::SetValue(const StatisticValue& v) {
	hasValue = true;
	value = v.vector;
	hasVariance = true;
	variance = v.variance;
}

bool SF::DataMsg::operator!=(const DataMsg& data) const {
	return !(operator==(data));
}

bool SF::DataMsg::operator==(const DataMsg& data) const {
	if (IsInvalid() != data.IsInvalid())
		return false;
	if (HasValue() != data.HasValue())
		return false;
	if (HasVariance() != data.HasVariance())
		return false;
	if (GetSourceID() != data.GetSourceID())
		return false;
	if (GetDataSourceType() != data.GetDataSourceType())
		return false;
	if (GetDataType() != data.GetDataType())
		return false;
	if (duration_since_epoch(GetTime()).count() != duration_since_epoch(data.GetTime()).count())
		return false;
	if (HasValue())
		if (!GetValue().isApprox(data.GetValue())) {
			std::cout << GetValue().transpose() << "  !=  " << data.GetValue().transpose() << std::endl;
			return false;
		}
	if (HasVariance())
		if (!GetVariance().isApprox(data.GetVariance()))
			return false;
	return true;
}
