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

void System::MeasurementDone(Eigen::VectorXd sensor_output) const {
	static Eigen::MatrixXd emptyMatrix;
	SetValues(StatisticValue(sensor_output), SystemValueType::OUTPUT);
}

void System::SetValues(StatisticValue value, SystemValueType type) const {
	// Check input sizes
	unsigned int n = getNumOf(type);
	if (value.Length() != n) {
		std::cout << "(System::SetValues) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	Call(value, type);
}



