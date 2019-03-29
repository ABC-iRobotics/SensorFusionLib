#pragma once

#include "Eigen/Dense"
#include <vector>


#include "StatisticValue.h"

#include "CallbackHandler.h"

enum SystemValueType { NOISE, DISTURBANCE, STATE, OUTPUT };

class System : public CallbackHandler<StatisticValue, SystemValueType>
{
public:
	System();
	~System();

	virtual unsigned int getNumOfStates() const = 0;

	virtual unsigned int getNumOfDisturbances() const = 0;

	virtual unsigned int getNumOfOutputs() const = 0;

	virtual unsigned int getNumOfNoises() const = 0;

	unsigned int getNumOf(SystemValueType type) const;

	virtual StatisticValue getInitializationStates() const;

	virtual StatisticValue getInitializationDisturbances() const;

	virtual StatisticValue getInitializationNoises() const;

	StatisticValue getInitValue(SystemValueType type) const;

	// Forward the actual output via the callbacks
	void MeasurementDone(Eigen::VectorXd sensor_output) const;

	// Forward the actual disturbance/noise properties, measured output, set manually the state, 
	void SetValues(StatisticValue value, SystemValueType type) const;
};

