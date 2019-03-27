#include "pch.h"
#include "System.h"
#include <iostream>

System::System() : vCallback(CallbackVector()) {
	static unsigned int IDcounter = 0;
	iID = IDcounter;
	IDcounter++;
}

System::~System()
{
}

unsigned int System::getNumOf(ValueType type) const {
	switch (type)
	{
	case System::NOISE:
		return getNumOfNoises();
		break;
	case System::DISTURBANCE:
		return getNumOfDisturbances();
		break;
	case System::STATE:
		return getNumOfStates();
		break;
	case System::OUTPUT:
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

StatisticValue System::getInitValue(ValueType type) const {
	switch (type)
	{
	case System::NOISE:
		return getInitializationNoises();
		break;
	case System::DISTURBANCE:
		return getInitializationDisturbances();
		break;
	case System::STATE:
		return getInitializationStates();
		break;
	default:
		return StatisticValue();
		break;
	}
}

void System::AddCallback(Callback callback, unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			std::cout << "Callback tried to be added more times (System.h)\n";
			return;
		}
	vCallback.push_back(Call(callback, ownerID));
}

void System::DeleteCallback(unsigned int ownerID) {
	for (unsigned int i = 0; i < vCallback.size(); i++)
		if (vCallback[i].ownerID == ownerID) {
			vCallback.erase(vCallback.begin() + i);
			return;
		}
	std::cout << "Not contained callback tried to be deleted (System.h)\n";
}

void System::MeasurementDone(Eigen::VectorXd sensor_output) const {
	static Eigen::MatrixXd emptyMatrix;
	SetValues(StatisticValue(sensor_output), ValueType::OUTPUT);
}

void System::SetValues(StatisticValue value, ValueType type) const {
	// Check input sizes
	unsigned int n = getNumOf(type);
	if (value.Length() != n) {
		std::cout << "(System::SetValues) Wrong value size! Returned without setting...\n";
		return;
	}
	// call the registered callbacks
	for (unsigned int i = 0; i < vCallback.size(); i++)
		vCallback[i].callback(value, type);
}



