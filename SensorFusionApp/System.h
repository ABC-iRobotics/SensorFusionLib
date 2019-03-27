#pragma once

#include "Eigen/Dense"
#include <vector>


#include "StatisticValue.h"

class System
{
	unsigned int iID;
public:
	System();
	~System();

	unsigned int getID() const { return iID; }

	virtual unsigned int getNumOfStates() const = 0;

	virtual unsigned int getNumOfDisturbances() const = 0;

	virtual unsigned int getNumOfOutputs() const = 0;

	virtual unsigned int getNumOfNoises() const = 0;

	enum ValueType { NOISE, DISTURBANCE, STATE, OUTPUT };

	unsigned int getNumOf(ValueType type) const;

	virtual StatisticValue getInitializationStates() const;

	virtual StatisticValue getInitializationDisturbances() const;

	virtual StatisticValue getInitializationNoises() const;

	StatisticValue getInitValue(ValueType type) const;

private:
	typedef std::function<void(StatisticValue value, ValueType type)> Callback;

	struct Call {
		Callback callback;
		unsigned int ownerID;
		Call(Callback call, unsigned int ID) : callback(call), ownerID(ID) {};
	};

	typedef std::vector<Call> CallbackVector;

	CallbackVector vCallback;

public:
	void AddCallback(Callback callback, unsigned int ownerID);

	void DeleteCallback(unsigned int ownerID);

	// Forward the actual output via the callbacks
	void MeasurementDone(Eigen::VectorXd sensor_output) const;

	// Forward the actual disturbance/noise properties, measured output, set manually the state, 
	void SetValues(StatisticValue value, ValueType type) const;
};

