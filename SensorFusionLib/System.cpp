#include "pch.h"
#include "System.h"
#include <iostream>

System::System() {}

System::~System()
{
}

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
/*
StatisticValue System::getInitializationStates() const {
	return StatisticValue(getNumOfStates());
}

StatisticValue System::getInitializationDisturbances() const {
	return StatisticValue(getNumOfDisturbances());
}

StatisticValue System::getInitializationNoises() const {
	unsigned int a = getNumOfNoises();
	return StatisticValue(a);
}

StatisticValue System::getInitValue(SystemValueType type) const {
	switch (type)
	{
	case SystemValueType::NOISE:
		return getInitializationNoises();
		break;
	case SystemValueType::DISTURBANCE:
		return getInitializationDisturbances();
		break;
	case SystemValueType::STATE:
		return getInitializationStates();
		break;
	default:
		return StatisticValue();
		break;
	}
}
*/
void System::MeasurementDone(Eigen::VectorXd sensor_output) const {
	if (sensor_output.size() != getNumOfOutputs()) {
		std::cout << "(System::MeasurementDone) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(sensor_output, EmptyClass());
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
}
SystemValueType System::getOutputValueType(System::UpdateType outType) {
	switch (outType) {
	case TIMEUPDATE:
		return SystemValueType::STATE;
	case MEASUREMENTUPDATE:
		return SystemValueType::OUTPUT;
	}
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



