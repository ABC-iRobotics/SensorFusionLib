#pragma once

#include "StatisticValue.h"
#include "CallbackHandler.h"
#include <string>

enum SystemValueType { NOISE, DISTURBANCE, STATE, OUTPUT };

struct SystemCallData {
	Eigen::VectorXd value;
	Eigen::MatrixXd variance;
	SystemValueType signalType;
	enum {VALUE, VARIANCE} valueType;
	SystemCallData(Eigen::VectorXd value, SystemValueType type);;
	SystemCallData(Eigen::MatrixXd variance, SystemValueType type);;
};

class System : public CallbackHandler<SystemCallData>
{
public:
	System();
	~System();

	virtual unsigned int getNumOfStates() const = 0;

	virtual unsigned int getNumOfDisturbances() const = 0;

	virtual unsigned int getNumOfOutputs() const = 0;

	virtual unsigned int getNumOfNoises() const = 0;

	unsigned int getNumOf(SystemValueType type) const;

	// Forward the actual output via the callbacks
	void MeasurementDone(Eigen::VectorXd sensor_output) const;

	// Set changed disturbance characteristics
	void SetDisturbanceValue(Eigen::VectorXd value) const;

	// Set changed disturbance characteristics
	void SetDisturbanceVariance(Eigen::MatrixXd value) const;

	// Set changed noise characteristics
	void SetNoiseValue(Eigen::VectorXd value) const;

	// Set changed noise characteristics
	void SetNoiseVariance(Eigen::MatrixXd value) const;

	virtual std::vector<std::string> getStateNames() const;

	virtual std::vector<std::string> getNoiseNames() const;

	virtual std::vector<std::string> getDisturbanceNames() const;

	virtual std::vector<std::string> getOutputNames() const;

	std::vector<std::string> getNames(SystemValueType type) const;

	virtual std::string getName() const;

	typedef std::shared_ptr<System> SystemPtr;

	enum UpdateType { TIMEUPDATE, MEASUREMENTUPDATE };

	enum InputType { STATE, INPUT };

	static SystemValueType getInputValueType(System::UpdateType outType, System::InputType inType);

	static SystemValueType getOutputValueType(System::UpdateType outType);

protected:
	void _systemTest() const;
};