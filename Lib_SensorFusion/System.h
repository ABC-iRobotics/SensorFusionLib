#pragma once
#include "defs.h"
#include "StatisticValue.h"
#include <string>

enum VariableType { VAR_STATE, VAR_EXTERNAL };

struct SystemCallData {
	Eigen::VectorXd value;
	Eigen::MatrixXd variance;
	DataType signalType;
	ValueType valueType;
	SystemCallData(Eigen::VectorXd value, DataType type);
	SystemCallData(Eigen::MatrixXd variance, DataType type);
};

class System {
	unsigned int ID;
public:
	unsigned int getID() const { return ID; }

	// Functions to override
	virtual unsigned int getNumOfStates() const = 0;

	virtual unsigned int getNumOfDisturbances() const = 0;

	virtual unsigned int getNumOfOutputs() const = 0;

	virtual unsigned int getNumOfNoises() const = 0;

	virtual std::vector<std::string> getStateNames() const;

	virtual std::vector<std::string> getNoiseNames() const;

	virtual std::vector<std::string> getDisturbanceNames() const;

	virtual std::vector<std::string> getOutputNames() const;

	virtual std::string getName() const;

	// Functions defined (based on them)
	System(unsigned int ID);

	~System();

	unsigned int getNumOf(DataType type) const;

	std::vector<std::string> getNames(DataType type) const;

	typedef std::shared_ptr<System> SystemPtr;

	static DataType getInputValueType(TimeUpdateType outType, VariableType inType);

	static DataType getOutputValueType(TimeUpdateType outType);

protected:
	void _systemTest() const;
};