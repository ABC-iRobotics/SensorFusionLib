#pragma once

#include "StatisticValue.h"
#include "CallbackHandler.h"
#include <string>

enum SystemValueType { NOISE, DISTURBANCE, STATE, OUTPUT };

class EmptyClass {
public:
	EmptyClass() {}
};

class System : public CallbackHandler <Eigen::VectorXd, EmptyClass>
{
public:
/*	struct SystemState {
		StatisticValue state;
		StatisticValue noise;
		StatisticValue disturbance;

		SystemState(const System& sys_) : state(sys_.getNumOfStates()),
			noise(sys_.getNumOfNoises()), disturbance(sys_.getNumOfDisturbances()) {}

		SystemState(StatisticValue state_, StatisticValue noise_, 
			StatisticValue disturbance_) : state(state_),
			noise(noise_), disturbance(disturbance_) {}
	};*/


	System();
	~System();

	virtual unsigned int getNumOfStates() const = 0;

	virtual unsigned int getNumOfDisturbances() const = 0;

	virtual unsigned int getNumOfOutputs() const = 0;

	virtual unsigned int getNumOfNoises() const = 0;

	unsigned int getNumOf(SystemValueType type) const;

	// Forward the actual output via the callbacks
	void MeasurementDone(Eigen::VectorXd sensor_output) const;

	// Forward the actual disturbance/noise properties, measured output, set manually the state, 
	//void SetValues(StatisticValue value, SystemValueType type) const;

	virtual std::vector<std::string> getStateNames() const;

	virtual std::vector<std::string> getNoiseNames() const;

	virtual std::vector<std::string> getDisturbanceNames() const;

	virtual std::vector<std::string> getOutputNames() const;

	virtual std::string getName() const;

	typedef std::shared_ptr<System> SystemPtr;

	enum UpdateType { TIMEUPDATE, MEASUREMENTUPDATE };

	enum InputType { STATE, INPUT };

	static SystemValueType getInputValueType(System::UpdateType outType, System::InputType inType);

	static SystemValueType getOutputValueType(System::UpdateType outType);
};