#include "System.h"
#include <iostream>

System::System() {}

System::~System() {}

unsigned int System::getNumOf(DataType type) const {
	switch (type)
	{
	case DataType::NOISE:
		return getNumOfNoises();
		break;
	case DataType::DISTURBANCE:
		return getNumOfDisturbances();
		break;
	case DataType::STATE:
		return getNumOfStates();
		break;
	case DataType::OUTPUT:
		return getNumOfOutputs();
		break;
	default:
		std::cout << "(System::getNumOf) unknown type input! Returned 0.\n";
		return 0;
		break;
	}
}
void System::MeasurementDone(Eigen::VectorXd sensor_output) const {
	if (sensor_output.size() != getNumOfOutputs()) {
		std::cout << "(System::MeasurementDone) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(sensor_output,DataType::OUTPUT));
}

// Set changed disturbance characteristics

void System::SetDisturbanceValue(Eigen::VectorXd value) const {
	if (value.size() != getNumOfDisturbances()) {
		std::cout << "(System::SetDisturbanceValue) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, DataType::DISTURBANCE));
}

// Set changed disturbance characteristics

void System::SetDisturbanceVariance(Eigen::MatrixXd value) const {
	if (value.cols() != getNumOfDisturbances() || value.rows() != getNumOfDisturbances()) {
		std::cout << "(System::SetDisturbanceVariance) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, DataType::DISTURBANCE));
}

// Set changed noise characteristics

void System::SetNoiseValue(Eigen::VectorXd value) const {
	if (value.size() != getNumOfNoises()) {
		std::cout << "(System::SetNoiseValue) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, DataType::NOISE));
}



std::vector<std::string> list(std::string fp, unsigned int n) {
	std::vector<std::string> out = std::vector<std::string>();
	for (unsigned int i = 0; i < n; i++)
		out.push_back(fp + "_" + std::to_string(i+1));
	return out;
}

// Set changed noise characteristics



// Set changed noise characteristics

void System::SetNoiseVariance(Eigen::MatrixXd value) const {
	if (value.cols() != getNumOfNoises() || value.rows() != getNumOfNoises()) {
		std::cout << "(System::SetNoiseVariance) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, DataType::NOISE));
}

std::vector<std::string> System::getStateNames() const {
	return list("x", getNumOfStates());
}
std::vector<std::string> System::getNoiseNames() const {
	return list("v", getNumOfNoises());
}
std::vector<std::string> System::getDisturbanceNames() const {
	return list("w", getNumOfDisturbances());
}
std::vector<std::string> System::getOutputNames() const {
	return list("y", getNumOfOutputs());
}

std::vector<std::string> System::getNames(DataType type) const {
	switch (type)
	{
	case DataType::NOISE:
		return getNoiseNames();
	case DataType::DISTURBANCE:
		return getDisturbanceNames();
	case DataType::STATE:
		return getStateNames();
	case DataType::OUTPUT:
		return getOutputNames();
	}
	throw std::runtime_error(std::string("System::getNames(): Unhandled argument!\n"));
}
std::string System::getName() const {
	return "System";
}
DataType System::getInputValueType(TimeUpdateType outType, VariableType inType) {
	if (inType == VAR_STATE)
		return DataType::STATE;
	switch (outType) {
	case TimeUpdateType::OUTPUT_UPDATE:
		return DataType::NOISE;
	case TimeUpdateType::STATE_UPDATE:
		return DataType::DISTURBANCE;
	}
	throw std::runtime_error(std::string("System::getInputValueType(): Unknown input!"));
}
DataType System::getOutputValueType(TimeUpdateType outType) {
	switch (outType) {
	case STATE_UPDATE:
		return DataType::STATE;
	case OUTPUT_UPDATE:
		return DataType::OUTPUT;
	}
	throw std::runtime_error(std::string("System::getOutputValueType(): Unknown input!"));
}
void System::_systemTest() const {
	if (getNames(DataType::STATE).size() != getNumOf(DataType::STATE))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(DataType::OUTPUT).size() != getNumOf(DataType::OUTPUT))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(DataType::DISTURBANCE).size() != getNumOf(DataType::DISTURBANCE))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(DataType::NOISE).size() != getNumOf(DataType::NOISE))
		throw std::runtime_error(std::string("System::_systemTest()"));
}

SystemCallData::SystemCallData(Eigen::VectorXd value, DataType type) :
	value(value), signalType(type), valueType(VALUE) {}

SystemCallData::SystemCallData(Eigen::MatrixXd variance, DataType type) :
	variance(variance), signalType(type), valueType(VARIANCE) {}
