#include "DataMsg.h"

#include "Time.h"
#include <iostream>

DataMsg::DataMsg(unsigned char ID, DataType type, OperationType source, unsigned long timestamp_in_us_) :
	sourceID(ID), dataType(type), dataSource(source), timestamp_in_us(timestamp_in_us_),
	empty(false), hasValue(false), hasVariance(false) {}

bool DataMsg::IsEmpty() const { return empty; }

bool DataMsg::HasValue() const { return hasValue; }

bool DataMsg::HasVariance() const { return hasVariance; }

Eigen::VectorXd DataMsg::GetValue() const { return value; }

Eigen::MatrixXd DataMsg::GetVariance() const { return variance; }

unsigned char DataMsg::GetSourceID() const { return sourceID; }

DataType DataMsg::GetDataType() const { return dataType; }

OperationType DataMsg::GetDataSourceType() const { return dataSource; }

void DataMsg::print() const {
	auto t = Time();
	if (IsEmpty()) {
		printf("EMTPY DataMsg.\n\n");
		return;
	}
	printf("SourceID: %d Content: ", sourceID);
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
		printf("filter(time update)");
		break;
	case FILTER_MEAS_UPDATE:
		printf("filter(meas. update)");
		break;
	case FILTER_PARAM_ESTIMATION:
		printf("filter(param. estimation)");
		break;
	case SENSOR:
		printf("sensor");
		break;
	default:
		break;
	}
	printf("\n Age: %f [ms]\n", (t - Time(timestamp_in_us)).TimeInS());
	if (hasValue)
		std::cout << "Value:\n" << value << std::endl;
	if (hasVariance)
		std::cout << "Variance:\n" << variance << std::endl;
	printf("\n");
}

DataMsg::DataMsg() // to initialize empty instances
	: empty(true), hasValue(false), hasVariance(false) {}

void DataMsg::SetVarianceMatrix(const Eigen::MatrixXd & m) {
	hasVariance = true;
	variance = m;
}

void DataMsg::SetValueVector(const Eigen::VectorXd & v) {
	hasValue = true;
	value = v;
}
