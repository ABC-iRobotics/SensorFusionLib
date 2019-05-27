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
	Call(sensor_output);
}
std::vector<std::string> list(std::string fp, unsigned int n) {
	std::vector<std::string> out = std::vector<std::string>();
	for (unsigned int i = 0; i < n; i++)
		out.push_back(fp + "_" + std::to_string(i+1));
	return out;
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
/*
void System::SetValues(StatisticValue value, SystemValueType type) const {
	// Check input sizes
	unsigned int n = getNumOf(type);
	if (value.Length() != n) {
		std::cout << "(System::SetValues) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(value, type);
}*/



