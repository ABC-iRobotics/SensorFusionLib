#include "System.h"
#include <iostream>

System::System() {}

System::~System() {}

unsigned int System::getNumOf(SystemValueType type) const {
	switch (type)
	{
	case SystemValueType::NOISE:
		return getNumOfNoises();
		break;
	case SystemValueType::DISTURBANCE:
		return getNumOfDisturbances();
		break;
	case SystemValueType::STATE:
		return getNumOfStates();
		break;
	case SystemValueType::OUTPUT:
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
	Call(SystemCallData(sensor_output,SystemValueType::OUTPUT));
}

// Set changed disturbance characteristics

void System::SetDisturbanceValue(Eigen::VectorXd value) const {
	if (value.size() != getNumOfDisturbances()) {
		std::cout << "(System::SetDisturbanceValue) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, SystemValueType::DISTURBANCE));
}

// Set changed disturbance characteristics

void System::SetDisturbanceVariance(Eigen::MatrixXd value) const {
	if (value.cols() != getNumOfDisturbances() || value.rows() != getNumOfDisturbances()) {
		std::cout << "(System::SetDisturbanceVariance) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, SystemValueType::DISTURBANCE));
}

// Set changed noise characteristics

void System::SetNoiseValue(Eigen::VectorXd value) const {
	if (value.size() != getNumOfNoises()) {
		std::cout << "(System::SetNoiseValue) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(SystemCallData(value, SystemValueType::NOISE));
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
	Call(SystemCallData(value, SystemValueType::NOISE));
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

std::vector<std::string> System::getNames(SystemValueType type) const {
	switch (type)
	{
	case SystemValueType::NOISE:
		return getNoiseNames();
	case SystemValueType::DISTURBANCE:
		return getDisturbanceNames();
	case SystemValueType::STATE:
		return getStateNames();
	case SystemValueType::OUTPUT:
		return getOutputNames();
	}
	throw std::runtime_error(std::string("System::getNames(): Unhandled argument!\n"));
}
std::string System::getName() const {
	return "System";
}
SystemValueType System::getInputValueType(System::UpdateType outType, System::InputType inType) {
	if (inType == STATE)
		return SystemValueType::STATE;
	switch (outType) {
	case UpdateType::MEASUREMENTUPDATE:
		return SystemValueType::NOISE;
	case UpdateType::TIMEUPDATE:
		return SystemValueType::DISTURBANCE;
	}
	throw std::runtime_error(std::string("System::getInputValueType(): Unknown input!"));
}
SystemValueType System::getOutputValueType(System::UpdateType outType) {
	switch (outType) {
	case TIMEUPDATE:
		return SystemValueType::STATE;
	case MEASUREMENTUPDATE:
		return SystemValueType::OUTPUT;
	}
	throw std::runtime_error(std::string("System::getOutputValueType(): Unknown input!"));
}
void System::_systemTest() const {
	if (getNames(SystemValueType::STATE).size() != getNumOf(SystemValueType::STATE))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(SystemValueType::OUTPUT).size() != getNumOf(SystemValueType::OUTPUT))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(SystemValueType::DISTURBANCE).size() != getNumOf(SystemValueType::DISTURBANCE))
		throw std::runtime_error(std::string("System::_systemTest()"));
	if (getNames(SystemValueType::NOISE).size() != getNumOf(SystemValueType::NOISE))
		throw std::runtime_error(std::string("System::_systemTest()"));
}

SystemCallData::SystemCallData(Eigen::VectorXd value, SystemValueType type) :
	value(value), signalType(type), valueType(VALUE) {}

SystemCallData::SystemCallData(Eigen::MatrixXd variance, SystemValueType type) :
	variance(variance), signalType(type), valueType(VARIANCE) {}
